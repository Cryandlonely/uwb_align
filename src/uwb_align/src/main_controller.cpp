#include <rclcpp/rclcpp.hpp>
#include "uwb_align/uwb_align_node.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uwb_align::UwbAlignController>());
    rclcpp::shutdown();
    return 0;
}
