#ifndef MYAGV_H
#define MYAGV_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>
#include <iostream>
#include <vector>



//#define sampleFreq	20.5f				// sample frequency in Hz
#define twoKpDef	1.0f				// (2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	0.0f				// (2.0f * 0.0f)	// 2 * integral gain

#define OFFSET_COUNT 	200

class MyAGV : public rclcpp::Node
{
public:
	MyAGV();
	~MyAGV();
	bool init();
	float invSqrt(float number);
	bool execute(double linearX, double linearY, double angularZ);
    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	void accelerometerOffset(float gx, float gy, float gz);
	void publisherOdom();
	void publisherImuSensor();
	void publisherImuSensorRaw();

private:
	bool readSpeed();
	void writeSpeed(double movex, double movey, double rot);
	void restore();
	void restoreRun();

	rclcpp::Time currentTime, lastTime;

	double x;
	double y;
	double theta;

	double vx;
	double vy;
	double vtheta;

	double ax;
	double ay;
	double az;

	double wx;
	double wy;
	double wz;
    
	float Gyroscope_Xdata_Offset;
	float Gyroscope_Ydata_Offset;
	float Gyroscope_Zdata_Offset;
	float sampleFreq;
	unsigned short Offest_Count;
    sensor_msgs::msg::Imu imu_data;
	rclcpp::Node n;
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_odom,pub_v,pub_imu,pub,pub_imu_raw;
	tf2_ros::TransformBroadcaster odomBroadcaster;
};


#endif // !MYAGV_H
