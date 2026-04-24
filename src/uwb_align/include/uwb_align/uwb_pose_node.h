#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "uwb_align/uwb_serial.h"

#include <memory>
#include <mutex>

namespace uwb_align {

/**
 * @brief UWB 三边定位节点
 *
 * 从串口读取 TOF Report Message，计算标签相对于双基站连线中点的位置，
 * 发布为 geometry_msgs/PoseWithCovarianceStamped，供 robot_localization
 * EKF 节点融合。
 *
 * 坐标定义（对准目标坐标系 "alignment_frame"）
 * ------------------------------------------
 *   基站 A: (-W/2, 0)      基站 B: (+W/2, 0)
 *   中点:   (  0,  0)
 *
 *   tag 当前位置: (x, y)
 *     x = (r_B² - r_A²) / (2·W)       —— 横向偏移，正值偏向基站 B 侧
 *     y = sqrt(((r_A+r_B)/2)² - x²)   —— 前向距离（到中点）
 *
 * 发布话题
 * --------
 *   /uwb/pose   geometry_msgs/PoseWithCovarianceStamped
 *   /uwb/debug  std_msgs/Float32MultiArray  [r_A(m), r_B(m), x(m), y(m)]
 */
class UwbPoseNode : public rclcpp::Node {
public:
    explicit UwbPoseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UwbPoseNode() override;

private:
    void DeclareAndLoadParams();
    void OnUwbRange(const TofRange& range);

    std::unique_ptr<UwbSerial> uwb_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr              debug_pub_;

    // ---- 参数 ----
    std::string serial_port_  = "/dev/ttyUSB0";
    int         baud_rate_    = 460800;
    double      separation_m_ = 2.0;    ///< 两基站间距 W (m)
    int         range_idx_a_  = 0;      ///< 基站 A 对应的 RANGE 索引 (0-3)
    int         range_idx_b_  = 1;      ///< 基站 B 对应的 RANGE 索引 (0-3)
    double      cov_xy_       = 0.04;   ///< UWB x/y 测量方差 (m²), σ≈0.2m
    std::string frame_id_     = "alignment_frame"; ///< 发布的坐标系名称

    std::mutex mutex_;
};

}  // namespace uwb_align
