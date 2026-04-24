#include "uwb_align/uwb_align_node.h"

#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

namespace uwb_align {

// 将角度归一化到 (-π, π]
static double normalizeAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ============================================================
UwbAlignController::UwbAlignController(const rclcpp::NodeOptions& options)
    : Node("uwb_align_controller", options)
{
    RCLCPP_INFO(get_logger(), "=== UwbAlignController 初始化 ===");
    DeclareAndLoadParams();

    // ---- 订阅 EKF 输出 ----
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&UwbAlignController::OnOdometry, this, std::placeholders::_1));

    // ---- 发布 ----
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_  = create_publisher<std_msgs::msg::String>("/uwb_align/status", 10);

    // ---- 控制定时器 ----
    auto period = std::chrono::duration<double>(1.0 / control_freq_);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&UwbAlignController::ControlLoop, this));

    RCLCPP_INFO(get_logger(), "订阅: /odometry/filtered");
    RCLCPP_INFO(get_logger(), "发布: /cmd_vel, /uwb_align/status");
    RCLCPP_INFO(get_logger(),
        "参数: stop_dist=%.2fm align_thresh=%.3fm yaw_Kp=%.2f yaw_Kd=%.2f fwd_Kp=%.2f",
        stop_dist_m_, align_thresh_m_, yaw_Kp_, yaw_Kd_, fwd_Kp_);
}

UwbAlignController::~UwbAlignController()
{
    PublishStop();
}

void UwbAlignController::DeclareAndLoadParams()
{
    declare_parameter<double>("stop_dist_m",    0.5);
    declare_parameter<double>("align_thresh_m", 0.05);
    declare_parameter<double>("yaw_Kp",         1.5);
    declare_parameter<double>("yaw_Kd",         0.2);
    declare_parameter<double>("fwd_Kp",         0.4);
    declare_parameter<double>("max_linear",     0.3);
    declare_parameter<double>("max_angular",    0.8);
    declare_parameter<double>("control_freq",   20.0);
    declare_parameter<double>("data_timeout_s", 0.5);

    stop_dist_m_    = get_parameter("stop_dist_m").as_double();
    align_thresh_m_ = get_parameter("align_thresh_m").as_double();
    yaw_Kp_         = get_parameter("yaw_Kp").as_double();
    yaw_Kd_         = get_parameter("yaw_Kd").as_double();
    fwd_Kp_         = get_parameter("fwd_Kp").as_double();
    max_linear_     = get_parameter("max_linear").as_double();
    max_angular_    = get_parameter("max_angular").as_double();
    control_freq_   = get_parameter("control_freq").as_double();
    data_timeout_s_ = get_parameter("data_timeout_s").as_double();
}

// ---- EKF 里程计回调 ----
void UwbAlignController::OnOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 从四元数提取 yaw
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    std::lock_guard<std::mutex> lock(mutex_);
    pos_x_      = msg->pose.pose.position.x;
    pos_y_      = msg->pose.pose.position.y;
    cur_yaw_    = yaw;
    omega_z_    = msg->twist.twist.angular.z;   // EKF 同时维护角速度
    data_valid_ = true;
    last_data_time_ = now();
    time_init_  = true;
}

// ---- 控制循环（定时器驱动）----
void UwbAlignController::ControlLoop()
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 超时检测
    if (time_init_) {
        double dt = (now() - last_data_time_).seconds();
        if (dt > data_timeout_s_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "EKF 数据超时 (%.2fs)，急停", dt);
            PublishStop();
            data_valid_ = false;
            return;
        }
    }

    if (!data_valid_) {
        return;
    }

    const double x    = pos_x_;   // 横向偏移 (m)，右正左负
    const double y    = pos_y_;   // 前向距离 (m)
    const double yaw  = cur_yaw_; // 当前航向 (rad)，由 EKF 维护
    const double w_z  = omega_z_; // 当前角速度 (rad/s)

    // 已对准且到位
    if (y < stop_dist_m_ && std::fabs(x) < align_thresh_m_) {
        if (state_ != AlignState::ALIGNED) {
            state_ = AlignState::ALIGNED;
            RCLCPP_INFO(get_logger(),
                "✓ 对准完成: x=%.3fm, y=%.3fm", x, y);
        }
        PublishStop();
        PublishStatus("ALIGNED");
        return;
    }
    state_ = (std::fabs(x) > align_thresh_m_) ? AlignState::ROTATING
                                               : AlignState::ADVANCING;

    // 期望朝向: 指向中点 (0, 0) 的方向
    //   desired_yaw = atan2(-x, y)
    //     当 x>0（偏右），需要向左转，desired_yaw 为负
    //     当 x<0（偏左），需要向右转，desired_yaw 为正
    const double desired_yaw = std::atan2(-x, y);
    const double yaw_err     = normalizeAngle(desired_yaw - yaw);

    // PD 角速度控制
    double angular_z = yaw_Kp_ * yaw_err - yaw_Kd_ * w_z;
    angular_z = std::clamp(angular_z, -max_angular_, max_angular_);

    // 前进：仅当航向基本对准时才前进，避免斜向运动
    double linear_x = 0.0;
    if (std::fabs(yaw_err) < M_PI / 6.0) {  // 航向误差 < 30°
        linear_x = fwd_Kp_ * std::max(0.0, y - stop_dist_m_);
        linear_x *= std::cos(yaw_err);       // 投影到前进方向
        linear_x = std::clamp(linear_x, 0.0, max_linear_);
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = linear_x;
    cmd.angular.z = angular_z;
    cmd_vel_pub_->publish(cmd);

    // 状态字符串
    char buf[128];
    std::snprintf(buf, sizeof(buf),
        "state=%s x=%.3fm y=%.3fm yaw_err=%.2f° vx=%.2f wz=%.2f",
        (state_ == AlignState::ROTATING) ? "ROTATING" : "ADVANCING",
        x, y,
        yaw_err * 180.0 / M_PI,
        linear_x, angular_z);
    PublishStatus(buf);

    RCLCPP_DEBUG(get_logger(), "%s", buf);
}

void UwbAlignController::PublishStop()
{
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
}

void UwbAlignController::PublishStatus(const std::string& msg)
{
    std_msgs::msg::String s;
    s.data = msg;
    status_pub_->publish(s);
}

}  // namespace uwb_align
