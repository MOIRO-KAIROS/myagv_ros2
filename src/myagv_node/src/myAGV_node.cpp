#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "myagv_node/myAGV.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyAGV>());
    rclcpp::shutdown();
    return 0;
}