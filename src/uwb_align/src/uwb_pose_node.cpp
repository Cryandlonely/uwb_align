#include "uwb_align/uwb_pose_node.h"

#include <cmath>

namespace uwb_align {

UwbPoseNode::UwbPoseNode(const rclcpp::NodeOptions& options)
    : Node("uwb_pose_node", options)
{
    RCLCPP_INFO(get_logger(), "=== UwbPoseNode 初始化 ===");
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
        RCLCPP_ERROR(get_logger(), "无法打开串口 [%s]，请检查设备路径和权限", serial_port_.c_str());
        throw std::runtime_error("UWB 串口打开失败: " + serial_port_);
    }
    if (!uwb_->startReading()) {
        throw std::runtime_error("UWB 串口读取线程启动失败");
    }

    RCLCPP_INFO(get_logger(), "串口 [%s] @ %d bps 已打开", serial_port_.c_str(), baud_rate_);
    RCLCPP_INFO(get_logger(), "基站间距: %.2f m, RANGE索引: A=%d B=%d",
        separation_m_, range_idx_a_, range_idx_b_);
    RCLCPP_INFO(get_logger(), "发布: /uwb/pose (PoseWithCovarianceStamped), /uwb/debug");
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
    declare_parameter<int>        ("range_idx_a",  0);
    declare_parameter<int>        ("range_idx_b",  1);
    declare_parameter<double>     ("cov_xy",       0.04);
    declare_parameter<std::string>("frame_id",     "alignment_frame");

    serial_port_  = get_parameter("serial_port").as_string();
    baud_rate_    = get_parameter("baud_rate").as_int();
    separation_m_ = get_parameter("separation_m").as_double();
    range_idx_a_  = get_parameter("range_idx_a").as_int();
    range_idx_b_  = get_parameter("range_idx_b").as_int();
    cov_xy_       = get_parameter("cov_xy").as_double();
    frame_id_     = get_parameter("frame_id").as_string();
}

void UwbPoseNode::OnUwbRange(const TofRange& range)
{
    // 检查两基站数据是否均有效（mask 对应 bit 为 1）
    const uint8_t need_mask = static_cast<uint8_t>(
        (1u << range_idx_a_) | (1u << range_idx_b_));
    if ((range.mask & need_mask) != need_mask) {
        RCLCPP_DEBUG(get_logger(), "MASK=0x%02x，基站数据不完整，跳过", range.mask);
        return;
    }

    const double r_a = range.range_mm[range_idx_a_] * 1e-3;  // mm → m
    const double r_b = range.range_mm[range_idx_b_] * 1e-3;
    const double W   = separation_m_;

    // 三边定位
    // x = (r_B² - r_A²) / (2·W)   横向偏移，正值 = 偏向基站 B 侧
    const double x_lat = (r_b * r_b - r_a * r_a) / (2.0 * W);

    // r_avg = (r_a + r_b) / 2,  y = sqrt(r_avg² - x²)
    const double r_avg = 0.5 * (r_a + r_b);
    const double radicand = r_avg * r_avg - x_lat * x_lat;
    if (radicand < 0.0) {
        RCLCPP_WARN(get_logger(), "三边定位几何异常 (r_A=%.3f r_B=%.3f)，跳过", r_a, r_b);
        return;
    }
    const double y_fwd = std::sqrt(radicand);

    // ---- 发布位姿（alignment_frame）----
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp    = now();
    pose_msg.header.frame_id = frame_id_;

    // position.x = 横向偏移，position.y = 前向距离
    pose_msg.pose.pose.position.x = x_lat;
    pose_msg.pose.pose.position.y = y_fwd;
    pose_msg.pose.pose.position.z = 0.0;

    // orientation = identity（航向由 IMU 通过 EKF 维护）
    pose_msg.pose.pose.orientation.w = 1.0;
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;

    // 协方差矩阵（6x6，row-major）
    // 仅设置 x-x 和 y-y 项，其余保持为 0（不更新）
    pose_msg.pose.covariance.fill(0.0);
    pose_msg.pose.covariance[0]  = cov_xy_;   // x 方差
    pose_msg.pose.covariance[7]  = cov_xy_;   // y 方差
    pose_msg.pose.covariance[35] = 1e6;        // yaw 不由 UWB 约束（设为大值）

    pose_pub_->publish(pose_msg);

    // ---- 调试话题 [r_A, r_B, x, y] ----
    std_msgs::msg::Float32MultiArray dbg;
    dbg.data = {
        static_cast<float>(r_a),
        static_cast<float>(r_b),
        static_cast<float>(x_lat),
        static_cast<float>(y_fwd)
    };
    debug_pub_->publish(dbg);

    RCLCPP_DEBUG(get_logger(),
        "UWB: r_A=%.3fm r_B=%.3fm → x=%.3fm y=%.3fm", r_a, r_b, x_lat, y_fwd);
}

}  // namespace uwb_align
