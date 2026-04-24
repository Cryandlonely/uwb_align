#include <rclcpp/rclcpp.hpp>
#include "uwb_align/uwb_pose_node.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uwb_align::UwbPoseNode>());
    rclcpp::shutdown();
    return 0;
}
