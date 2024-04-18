#include "myagv_bringup/myAGV.hpp"
#include <iostream>

double linearX = 0.0;
double linearY = 0.0;
double angularZ = 0.0;

using std::placeholders::_1;

void cmdCallback(const geometry_msgs::msg::Twist& msg)
{
	linearX = msg.linear.x;
	linearY = msg.linear.y;
	angularZ = msg.angular.z;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MyAGV>();

	if (!node->init()) {
		RCLCPP_ERROR(node->get_logger(), "myAGV initialization failed!");
    } else {
        RCLCPP_INFO(node->get_logger(), "myAGV initialized successfully!");
    }
	auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 50, cmdCallback);

	rclcpp::Rate loop_rate(100); // 100Hz로 루프 주기 설정

    while (rclcpp::ok()) {
        rclcpp::spin_some(node); // 사용 가능한 모든 콜백 처리
        node->execute(linearX, linearY, angularZ); // 여기서 execute 함수가 올바른지 확인 필요
        loop_rate.sleep();
    }

    rclcpp::shutdown();

	return 0;
}
