#include "myagv_odometry/myAGV.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <memory>

double linearX = 0.0;
double linearY = 0.0;
double angularZ = 0.0;

class MyAGVNode : public rclcpp::Node
{
public:
    MyAGVNode() : Node("myagv_odometry_node")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 50,
            std::bind(&MyAGVNode::cmdCallback, this, std::placeholders::_1));
        myAGV_ = std::make_shared<MyAGV>();

        if (!myAGV_->init())
            RCLCPP_ERROR(this->get_logger(), "myAGV initialized failed!");
        else
            RCLCPP_INFO(this->get_logger(), "myAGV initialized successful!");
    }

private:
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linearX = msg->linear.x;
        linearY = msg->linear.y;
        angularZ = msg->angular.z;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::shared_ptr<MyAGV> myAGV_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyAGVNode>();

    rclcpp::Rate loop_rate(100);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->myAGV_->execute(linearX, linearY, angularZ);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
