#include "myagv_odometry/myAGV.hpp"
#include <iostream>
#include <memory>

double linearX = 0.0;
double linearY = 0.0;
double angularZ = 0.0;

using std::placeholders::_1;

class MyAGVSubscriber : public rclcpp::Node
{
public:
    MyAGVSubscriber() : Node("myagv_odometry_node"), myAGV(std::make_shared<MyAGV>())
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 50, std::bind(&MyAGVSubscriber::cmdCallback, this, _1));

        if (!myAGV->init())
            RCLCPP_ERROR(this->get_logger(), "myAGV initialized failed!");
        else
            RCLCPP_INFO(this->get_logger(), "myAGV initialized successful!");
    }

private:
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linearX = msg->linear.x;
        double linearY = msg->linear.y;
        double angularZ = msg->angular.z;

        // Process the incoming velocity commands
        myAGV->execute(linearX, linearY, angularZ);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::shared_ptr<MyAGV> myAGV;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyAGVSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
