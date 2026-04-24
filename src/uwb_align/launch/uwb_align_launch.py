import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    uwb_share    = FindPackageShare("uwb_align")
    livox_share  = FindPackageShare("livox_ros_driver2")
    chassis_share = FindPackageShare("chassis")

    params_file   = PathJoinSubstitution([uwb_share,  "config", "uwb_align_params.yaml"])
    ekf_cfg       = PathJoinSubstitution([uwb_share,  "config", "ekf.yaml"])
    chassis_cfg   = PathJoinSubstitution([chassis_share, "config", "chassis_params.yaml"])

    # ---- 可覆盖参数 ----
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="UWB 模块串口路径，例如 /dev/ttyUSB0")

    separation_arg = DeclareLaunchArgument(
        "separation_m",
        default_value="2.0",
        description="两基站间距（米）")

    # ---- 节点 1: Livox MID360 雷达驱动（发布 /livox/imu）----
    # 注意：启动前须确认 MID360_config.json 中的 IP 配置正确
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([livox_share, "launch_ROS2", "msg_MID360_launch.py"])
        ])
    )

    # ---- 节点 2: 底盘 CAN 控制（订阅 /cmd_vel）----
    chassis_node = Node(
        package="chassis",
        executable="chassis_node",
        name="chassis_node",
        output="screen",
        parameters=[chassis_cfg]
    )

    # ---- 节点 3: UWB 三边定位位姿发布 (/uwb/pose) ----
    # 前提：小车启动时需 大致正对 两基站中点，EKF yaw 以此方向为零点
    uwb_pose_node = Node(
        package="uwb_align",
        executable="uwb_pose_node",
        name="uwb_pose_node",
        output="screen",
        parameters=[
            params_file,
            {
                "serial_port":  LaunchConfiguration("serial_port"),
                "separation_m": LaunchConfiguration("separation_m"),
            }
        ]
    )

    # ---- 节点 4: robot_localization EKF（融合 /uwb/pose + /livox/imu）----
    # 输出 /odometry/filtered，包含平滑后的 (x_lat, y_fwd, yaw)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[ekf_cfg]
    )

    # ---- 节点 5: 对准控制器（订阅 /odometry/filtered，发布 /cmd_vel）----
    controller_node = Node(
        package="uwb_align",
        executable="uwb_align_controller",
        name="uwb_align_controller",
        output="screen",
        parameters=[params_file]
    )

    return LaunchDescription([
        serial_port_arg,
        separation_arg,
        livox_launch,
        chassis_node,
        uwb_pose_node,
        ekf_node,
        controller_node,
    ])
