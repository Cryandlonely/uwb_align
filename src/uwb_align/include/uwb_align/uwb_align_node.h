#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>

namespace uwb_align {

/**
 * @brief UWB 对准控制节点（两阶段版）
 *
 * ── 阶段一：UWB 对准（y > final_dist_m）──────────────────────────
 *   linear.x  = clamp(Kp_fwd * (y - final_dist_m), 0, max_linear)
 *               距离越近速越慢，到 final_dist_m 处速度自然减为 0
 *   angular.z = clamp(Kp_yaw * yaw_err - Kd_yaw * omega_z, ±max_angular)
 *   yaw_err   = normalizeAngle(atan2(x, y) - cur_yaw)
 *
 * ── 阶段二：IMU 恒速末段（y ≤ final_dist_m）─────────────────────
 *   进入此阶段时记录当前航向 final_yaw_，之后只用 IMU 保持直行
 *   linear.x  = linear_speed（恒定，默认 0.3 m/s）
 *   angular.z = clamp(Kp_yaw * yaw_err - Kd_yaw * omega_z, ±max_angular)
 *               yaw_err = normalizeAngle(final_yaw_ - cur_yaw)
 *   持续 final_duration_s 秒（= final_dist_m / linear_speed * 安全系数）后停止
 *
 * 话题
 * ----
 *   订阅: /odometry/filtered   nav_msgs/Odometry   (来自 robot_localization)
 *   发布: /cmd_vel              geometry_msgs/Twist
 *         /uwb_align/status    std_msgs/String
 */
class UwbAlignController : public rclcpp::Node {
public:
    explicit UwbAlignController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~UwbAlignController() override;

private:
    void DeclareAndLoadParams();
    void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void ControlLoop();
    void PublishStop();
    void PublishStatus(const std::string& msg);

    // ---- ROS2 ----
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      status_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ---- 参数 ----
    double final_dist_m_     = 1.0;   ///< UWB阶段结束/IMU阶段开始的前向距离 (m)
    double linear_speed_     = 0.3;   ///< IMU恒速阶段的前进速度 (m/s)
    double final_duration_s_ = 0.0;   ///< IMU阶段持续时间 (s)，0=自动计算
    double fwd_Kp_           = 0.4;   ///< UWB阶段前进 P 增益
    double max_linear_       = 0.4;   ///< UWB阶段最大线速度 (m/s)
    double yaw_Kp_           = 1.5;   ///< 航向误差 P 增益（两阶段共用）
    double yaw_Kd_           = 0.2;   ///< 航向误差 D 增益（两阶段共用）
    double max_angular_      = 0.8;   ///< 最大角速度 (rad/s)
    double control_freq_     = 20.0;  ///< 控制频率 (Hz)
    double data_timeout_s_   = 0.5;   ///< 数据超时急停阈值 (s)

    // ---- 状态（受 mutex_ 保护）----
    std::mutex mutex_;
    double pos_x_       = 0.0;   ///< 横向偏移 (m)
    double pos_y_       = 0.0;   ///< 前向距离 (m)
    double cur_yaw_     = 0.0;   ///< 当前航向角 (rad)
    double omega_z_     = 0.0;   ///< 当前角速度 (rad/s)
    bool   data_valid_  = false;
    rclcpp::Time last_data_time_;
    bool   time_init_   = false;

    // 阶段二状态
    double           final_yaw_         = 0.0;   ///< 进入IMU阶段时锁定的目标航向
    rclcpp::Time     final_start_time_;           ///< 进入IMU阶段的时刻
    bool             in_final_phase_    = false;  ///< 是否处于IMU恒速阶段

    enum class AlignState { UWB_ALIGN, IMU_FINAL, ALIGNED };
    AlignState state_ = AlignState::UWB_ALIGN;
};

}  // namespace uwb_align
