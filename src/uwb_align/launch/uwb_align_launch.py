from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("uwb_align")

    params_file = PathJoinSubstitution([pkg_share, "config", "uwb_align_params.yaml"])
    ekf_cfg     = PathJoinSubstitution([pkg_share, "config", "ekf.yaml"])

    # ---- 可覆盖参数 ----
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="UWB 模块串口路径，例如 /dev/ttyUSB0")

    separation_arg = DeclareLaunchArgument(
        "separation_m",
        default_value="2.0",
        description="两基站间距（米）")

    # ---- 节点 1: UWB 三边定位位姿发布 ----
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

    # ---- 节点 2: robot_localization EKF（融合 UWB + IMU）----
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[ekf_cfg],
        remappings=[
            # EKF 订阅 /livox/imu，输出 /odometry/filtered
            # 若 livox 驱动的 IMU 话题不同，在此修改:
            # ("/livox/imu", "/your/imu/topic"),
        ]
    )

    # ---- 节点 3: 对准控制器 ----
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
        uwb_pose_node,
        ekf_node,
        controller_node,
    ])
