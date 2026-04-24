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
        "参数: final_dist=%.1fm linear_speed=%.2fm/s final_duration=%.1fs"
        " fwd_Kp=%.2f max_linear=%.2f yaw_Kp=%.2f yaw_Kd=%.2f",
        final_dist_m_, linear_speed_, final_duration_s_,
        fwd_Kp_, max_linear_, yaw_Kp_, yaw_Kd_);
}

UwbAlignController::~UwbAlignController()
{
    PublishStop();
}

void UwbAlignController::DeclareAndLoadParams()
{
    declare_parameter<double>("final_dist_m",     1.0);
    declare_parameter<double>("linear_speed",     0.3);
    declare_parameter<double>("final_duration_s", 0.0);   // 0 = 自动计算
    declare_parameter<double>("fwd_Kp",           0.4);
    declare_parameter<double>("max_linear",       0.4);
    declare_parameter<double>("yaw_Kp",           1.5);
    declare_parameter<double>("yaw_Kd",           0.2);
    declare_parameter<double>("max_angular",      0.8);
    declare_parameter<double>("control_freq",     20.0);
    declare_parameter<double>("data_timeout_s",   0.5);

    final_dist_m_     = get_parameter("final_dist_m").as_double();
    linear_speed_     = get_parameter("linear_speed").as_double();
    final_duration_s_ = get_parameter("final_duration_s").as_double();
    fwd_Kp_           = get_parameter("fwd_Kp").as_double();
    max_linear_       = get_parameter("max_linear").as_double();
    yaw_Kp_           = get_parameter("yaw_Kp").as_double();
    yaw_Kd_           = get_parameter("yaw_Kd").as_double();
    max_angular_      = get_parameter("max_angular").as_double();
    control_freq_     = get_parameter("control_freq").as_double();
    data_timeout_s_   = get_parameter("data_timeout_s").as_double();

    // 自动计算 IMU 阶段持续时间：距离 / 速度 * 1.3 安全系数
    if (final_duration_s_ <= 0.0) {
        final_duration_s_ = (final_dist_m_ / linear_speed_) * 1.3;
    }
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

    const double x   = pos_x_;
    const double y   = pos_y_;
    const double yaw = cur_yaw_;
    const double w_z = omega_z_;

    // ================================================================
    // 阶段二：IMU 恒速末段（y 已进入 final_dist_m 范围）
    // ================================================================
    if (state_ == AlignState::IMU_FINAL) {
        double elapsed = (now() - final_start_time_).seconds();
        if (elapsed >= final_duration_s_) {
            state_ = AlignState::ALIGNED;
            RCLCPP_INFO(get_logger(),
                "IMU 末段完成 (%.1fs)，全程对准结束", elapsed);
            PublishStop();
            PublishStatus("ALIGNED");
            return;
        }

        // 维持进入末段时锁定的航向，恒速前进
        const double yaw_err = normalizeAngle(final_yaw_ - yaw);
        double angular_z = yaw_Kp_ * yaw_err - yaw_Kd_ * w_z;
        angular_z = std::clamp(angular_z, -max_angular_, max_angular_);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = linear_speed_;
        cmd.angular.z = angular_z;
        cmd_vel_pub_->publish(cmd);

        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "IMU_FINAL t=%.1f/%.1fs yaw_err=%.1f° vx=%.2f wz=%.2f",
            elapsed, final_duration_s_,
            yaw_err * 180.0 / M_PI, linear_speed_, angular_z);
        PublishStatus(buf);
        RCLCPP_DEBUG(get_logger(), "%s", buf);
        return;
    }

    // ALIGNED 状态保持停止
    if (state_ == AlignState::ALIGNED) {
        PublishStop();
        PublishStatus("ALIGNED");
        return;
    }

    // ================================================================
    // 阶段一：UWB 对准阶段（变速 + PD 转向）
    // ================================================================

    // 切换到阶段二
    if (y <= final_dist_m_) {
        state_            = AlignState::IMU_FINAL;
        final_yaw_        = yaw;
        final_start_time_ = now();
        RCLCPP_INFO(get_logger(),
            "切入 IMU 末段: y=%.3fm (≤%.1fm)，锁定航向 %.2f rad，"
            "恒速 %.2fm/s 持续 %.1fs",
            y, final_dist_m_, final_yaw_, linear_speed_, final_duration_s_);
        return;
    }
    state_ = AlignState::UWB_ALIGN;

    // 期望朝向：alignment_frame 中 yaw=0 即车头正对基站连线法线（y轴方向）
    // EKF yaw 已由 UWB 双标签直接测量，无需从位置几何反算
    const double desired_yaw = 0.0;
    const double yaw_err     = normalizeAngle(desired_yaw - yaw);

    double angular_z = yaw_Kp_ * yaw_err - yaw_Kd_ * w_z;
    angular_z = std::clamp(angular_z, -max_angular_, max_angular_);

    // 变速前进：距 final_dist_m 越近速越小，到达时自然归零
    double linear_x = fwd_Kp_ * (y - final_dist_m_);
    linear_x = std::clamp(linear_x, 0.0, max_linear_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = linear_x;
    cmd.angular.z = angular_z;
    cmd_vel_pub_->publish(cmd);

    char buf[128];
    std::snprintf(buf, sizeof(buf),
        "UWB_ALIGN x=%.3fm y=%.3fm yaw_err=%.1f° vx=%.2f wz=%.2f",
        x, y, yaw_err * 180.0 / M_PI, linear_x, angular_z);
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
