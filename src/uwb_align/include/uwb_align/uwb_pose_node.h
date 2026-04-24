#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "uwb_align/uwb_serial.h"

#include <memory>
#include <mutex>

namespace uwb_align {

/**
 * @brief UWB 双标签定位节点
 *
 * 硬件布局
 * ----------
 *   基站 A: (-W/2, 0)   基站 B: (+W/2, 0)   [alignment_frame]
 *   Tag0: 车头，安装在车头中轴线上
 *   Tag1: 车尾，安装在车尾中轴线上
 *   两标签轴距为 L（量测实际安装距离）
 *
 * 解算原理
 * ----------
 *   1. 分别对 Tag0、Tag1 做三边定位：
 *        x_i = (r_Ai² - r_Bi²) / (2W)
 *        y_i = sqrt((r_Ai²+r_Bi²)/2 - x_i² - (W/2)²)
 *
 *   2. 车体中心位置：
 *        x_c = (x0 + x1) / 2
 *        y_c = (y0 + y1) / 2
 *
 *   3. 车头到车尾向量在 alignment_frame 中的航向角：
 *        theta = atan2(x0 - x1, y0 - y1)
 *
 * 发布话题
 * --------
 *   /uwb/pose   geometry_msgs/PoseWithCovarianceStamped
 *   /uwb/debug  std_msgs/Float32MultiArray
 *               [x0, y0, x1, y1, x_c, y_c, theta_deg]
 */
class UwbPoseNode : public rclcpp::Node {
public:
    explicit UwbPoseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UwbPoseNode() override;

private:
    void DeclareAndLoadParams();
    void OnUwbRange(const TofRange& range);

    /// 三边定位，返回 false 表示几何异常
    static bool trilaterate(double r_a, double r_b, double W,
                            double& x_out, double& y_out);

    std::unique_ptr<UwbSerial> uwb_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr              debug_pub_;

    // ---- 参数 ----
    std::string serial_port_  = "/dev/ttyUSB0";
    int         baud_rate_    = 460800;
    double      separation_m_ = 2.0;    ///< 两基站间距 W (m)
    double      tag_dist_m_   = 0.5;    ///< 两标签轴距 L (m)
    int         range_idx_a_  = 0;      ///< 基站 A 对应的 RANGE 索引 (0-3)
    int         range_idx_b_  = 1;      ///< 基站 B 对应的 RANGE 索引 (0-3)
    int         tag0_id_      = 0;      ///< 车头标签 short ID
    int         tag1_id_      = 1;      ///< 车尾标签 short ID
    double      cov_xy_       = 0.04;   ///< UWB x/y 测量方差 (m²)
    double      cov_yaw_      = 0.02;   ///< UWB 航向方差 (rad²)
    double      data_age_s_   = 0.1;    ///< 两帧数据允许的最大时间差 (s)
    std::string frame_id_     = "alignment_frame";

    // ---- 双标签数据缓存 ----
    struct TagData {
        double r_a = 0.0;
        double r_b = 0.0;
        rclcpp::Time stamp;
        bool valid = false;
    };
    std::mutex mutex_;
    TagData tag0_data_;   ///< 车头标签最新数据
    TagData tag1_data_;   ///< 车尾标签最新数据
};

}  // namespace uwb_align
