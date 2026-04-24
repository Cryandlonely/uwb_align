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
 * @brief UWB 对准控制节点（EKF 融合版）
 *
 * 订阅 robot_localization ekf_node 输出的 /odometry/filtered，
 * 其中包含经过 EKF 融合（UWB 位置 + IMU 角速度）后的平滑估计：
 *   pose.position.x  —— 横向偏移（相对于双基站中点，右正左负）
 *   pose.position.y  —— 前向距离（到中点的距离）
 *   pose.orientation —— 当前航向（由 IMU 角速度积分得到）
 *   twist.angular.z  —— 当前航向角速度（来自 IMU）
 *
 * 控制律
 * ------
 *   desired_yaw   = atan2(-x, y)          # 指向目标中点所需朝向
 *   yaw_error     = desired_yaw - current_yaw
 *   angular.z     = clamp(Kp_yaw * yaw_error - Kd_yaw * omega_z, ±max_angular)
 *   linear.x      = clamp(Kp_fwd * (y - stop_dist) * cos(yaw_error), 0, max_linear)
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
    double stop_dist_m_     = 0.5;   ///< 前向距离小于此值则停止 (m)
    double align_thresh_m_  = 0.05;  ///< 横向误差小于此值视为对准完成 (m)
    double yaw_Kp_          = 1.5;   ///< 航向误差 P 增益
    double yaw_Kd_          = 0.2;   ///< 航向误差 D 增益（抑制振荡）
    double fwd_Kp_          = 0.4;   ///< 前进 P 增益
    double max_linear_      = 0.3;   ///< 最大线速度 (m/s)
    double max_angular_     = 0.8;   ///< 最大角速度 (rad/s)
    double control_freq_    = 20.0;  ///< 控制频率 (Hz)
    double data_timeout_s_  = 0.5;   ///< 数据超时急停阈值 (s)

    // ---- 状态（受 mutex_ 保护）----
    std::mutex mutex_;
    double pos_x_       = 0.0;   ///< 横向偏移 (m)
    double pos_y_       = 0.0;   ///< 前向距离 (m)
    double cur_yaw_     = 0.0;   ///< 当前航向角 (rad)
    double omega_z_     = 0.0;   ///< 当前角速度 (rad/s)
    bool   data_valid_  = false;
    rclcpp::Time last_data_time_;
    bool   time_init_   = false;

    enum class AlignState { ROTATING, ADVANCING, ALIGNED };
    AlignState state_ = AlignState::ROTATING;
};

}  // namespace uwb_align
