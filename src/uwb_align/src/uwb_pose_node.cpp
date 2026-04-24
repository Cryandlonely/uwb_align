#include "uwb_align/uwb_pose_node.h"

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

namespace uwb_align {

UwbPoseNode::UwbPoseNode(const rclcpp::NodeOptions& options)
    : Node("uwb_pose_node", options)
{
    RCLCPP_INFO(get_logger(), "=== UwbPoseNode (双标签) 初始化 ===");
    DeclareAndLoadParams();

    // ---- 发布 ----
    pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/uwb/pose", 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/uwb/debug", 10);

    // ---- 串口 ----
    uwb_ = std::make_unique<UwbSerial>();
    uwb_->setCallback([this](const TofRange& r) { OnUwbRange(r); });

    if (!uwb_->open(serial_port_, baud_rate_)) {
        RCLCPP_ERROR(get_logger(), "无法打开串口 [%s]", serial_port_.c_str());
        throw std::runtime_error("UWB 串口打开失败: " + serial_port_);
    }
    if (!uwb_->startReading()) {
        throw std::runtime_error("UWB 串口读取线程启动失败");
    }

    RCLCPP_INFO(get_logger(), "串口 [%s] @ %d bps 已打开", serial_port_.c_str(), baud_rate_);
    RCLCPP_INFO(get_logger(),
        "基站间距: %.2f m, RANGE索引: A=%d B=%d, Tag0(头)=%d Tag1(尾)=%d, 轴距=%.2f m",
        separation_m_, range_idx_a_, range_idx_b_, tag0_id_, tag1_id_, tag_dist_m_);
}

UwbPoseNode::~UwbPoseNode()
{
    if (uwb_) {
        uwb_->stopReading();
        uwb_->close();
    }
}

void UwbPoseNode::DeclareAndLoadParams()
{
    declare_parameter<std::string>("serial_port",  "/dev/ttyUSB0");
    declare_parameter<int>        ("baud_rate",    460800);
    declare_parameter<double>     ("separation_m", 2.0);
    declare_parameter<double>     ("tag_dist_m",   0.5);
    declare_parameter<int>        ("range_idx_a",  0);
    declare_parameter<int>        ("range_idx_b",  1);
    declare_parameter<int>        ("tag0_id",      0);
    declare_parameter<int>        ("tag1_id",      1);
    declare_parameter<double>     ("cov_xy",       0.04);
    declare_parameter<double>     ("cov_yaw",      0.02);
    declare_parameter<double>     ("data_age_s",   0.1);
    declare_parameter<std::string>("frame_id",     "alignment_frame");

    serial_port_  = get_parameter("serial_port").as_string();
    baud_rate_    = get_parameter("baud_rate").as_int();
    separation_m_ = get_parameter("separation_m").as_double();
    tag_dist_m_   = get_parameter("tag_dist_m").as_double();
    range_idx_a_  = get_parameter("range_idx_a").as_int();
    range_idx_b_  = get_parameter("range_idx_b").as_int();
    tag0_id_      = static_cast<int>(get_parameter("tag0_id").as_int());
    tag1_id_      = static_cast<int>(get_parameter("tag1_id").as_int());
    cov_xy_       = get_parameter("cov_xy").as_double();
    cov_yaw_      = get_parameter("cov_yaw").as_double();
    data_age_s_   = get_parameter("data_age_s").as_double();
    frame_id_     = get_parameter("frame_id").as_string();
}

bool UwbPoseNode::trilaterate(double r_a, double r_b, double W,
                               double& x_out, double& y_out)
{
    // 基站 A: (-W/2, 0)，基站 B: (+W/2, 0)
    // x = (r_A² - r_B²) / (2W)
    // y = sqrt((r_A²+r_B²)/2 - x² - (W/2)²)
    x_out = (r_a * r_a - r_b * r_b) / (2.0 * W);
    const double radicand = 0.5 * (r_a * r_a + r_b * r_b)
                            - x_out * x_out
                            - (W * 0.5) * (W * 0.5);
    if (radicand <= 0.0) {
        return false;
    }
    y_out = std::sqrt(radicand);
    return true;
}

void UwbPoseNode::OnUwbRange(const TofRange& range)
{
    // 检查两基站数据是否均有效
    const uint8_t need_mask = static_cast<uint8_t>(
        (1u << range_idx_a_) | (1u << range_idx_b_));
    if ((range.mask & need_mask) != need_mask) {
        RCLCPP_DEBUG(get_logger(), "MASK=0x%02x 不完整，跳过", range.mask);
        return;
    }

    const double r_a = range.range_mm[range_idx_a_] * 1e-3;
    const double r_b = range.range_mm[range_idx_b_] * 1e-3;
    const int    tid = range.tag_id;

    std::lock_guard<std::mutex> lk(mutex_);

    // 缓存对应标签数据
    if (tid == tag0_id_) {
        tag0_data_.r_a   = r_a;
        tag0_data_.r_b   = r_b;
        tag0_data_.stamp = now();
        tag0_data_.valid = true;
    } else if (tid == tag1_id_) {
        tag1_data_.r_a   = r_a;
        tag1_data_.r_b   = r_b;
        tag1_data_.stamp = now();
        tag1_data_.valid = true;
    } else {
        RCLCPP_DEBUG(get_logger(), "未知 tag_id=%d，跳过", tid);
        return;
    }

    // 两标签数据都有效且时间差在阈值内才联合解算
    if (!tag0_data_.valid || !tag1_data_.valid) return;

    const double age = std::abs(
        (tag0_data_.stamp - tag1_data_.stamp).seconds());
    if (age > data_age_s_) {
        RCLCPP_DEBUG(get_logger(), "两标签时间差 %.3fs > %.3fs，等待刷新", age, data_age_s_);
        return;
    }

    // 三边定位
    double x0, y0, x1, y1;
    if (!trilaterate(tag0_data_.r_a, tag0_data_.r_b, separation_m_, x0, y0)) {
        RCLCPP_WARN(get_logger(), "Tag0 三边定位几何异常，跳过");
        return;
    }
    if (!trilaterate(tag1_data_.r_a, tag1_data_.r_b, separation_m_, x1, y1)) {
        RCLCPP_WARN(get_logger(), "Tag1 三边定位几何异常，跳过");
        return;
    }

    // 车体中心
    const double x_c = 0.5 * (x0 + x1);
    const double y_c = 0.5 * (y0 + y1);

    // 车头方向（Tag0=车头，Tag1=车尾）
    // theta: 车头 alignment_frame y轴方向的偏差（正值偏右）
    const double theta = std::atan2(x0 - x1, y0 - y1);

    // 发布位姿
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp    = now();
    pose_msg.header.frame_id = frame_id_;

    pose_msg.pose.pose.position.x = x_c;
    pose_msg.pose.pose.position.y = y_c;
    pose_msg.pose.pose.position.z = 0.0;

    // 绕 Z 轴旋转 theta -> 四元数
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    pose_msg.pose.covariance.fill(0.0);
    pose_msg.pose.covariance[0]  = cov_xy_;   // x
    pose_msg.pose.covariance[7]  = cov_xy_;   // y
    pose_msg.pose.covariance[35] = cov_yaw_;  // yaw

    pose_pub_->publish(pose_msg);

    // 调试话题 [x0,y0, x1,y1, x_c,y_c, theta_deg]
    std_msgs::msg::Float32MultiArray dbg;
    dbg.data = {
        static_cast<float>(x0),   static_cast<float>(y0),
        static_cast<float>(x1),   static_cast<float>(y1),
        static_cast<float>(x_c),  static_cast<float>(y_c),
        static_cast<float>(theta * 180.0 / M_PI)
    };
    debug_pub_->publish(dbg);

    RCLCPP_DEBUG(get_logger(),
        "UWB: Tag0=(%.2f,%.2f) Tag1=(%.2f,%.2f) → center=(%.2f,%.2f) θ=%.1f°",
        x0, y0, x1, y1, x_c, y_c, theta * 180.0 / M_PI);
}

}  // namespace uwb_align
