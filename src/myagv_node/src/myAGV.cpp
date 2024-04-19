#include "myagv_node/myAGV.hpp" 

#include <vector>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <time.h>

const unsigned char header[2] = { 0xfe, 0xfe };

boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyAMA1");

std::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9} };
std::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9} };

void send(){
    ;
}

void receive(){
    ;
}

MyAGV::MyAGV()
: Node("myagv_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(this->get_logger(), "myAGV_node started");
    std::cout << "myAGV_node started" << std::endl;
    x = 0.0;
    y = 0.0;
    theta = 0.0;

    vx = 0.0;
    vy = 0.0;
    vtheta = 0.0;

    ax = 0.0;
    ay = 0.0;
    az = 0.0;

    wx = 0.0;
    wy = 0.0;
    wz = 0.0;

    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

MyAGV::~MyAGV()
{
    ;
}

bool MyAGV::init()
{
    std::cout << "MyAGV Initailizing!" << std::endl;
    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    sp.set_option(boost::asio::serial_port::character_size(8));

    // Current & Last time update
    currentTime = rclcpp::Clock().now();
    lastTime = rclcpp::Clock().now();
    
    // pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    // pub_v = n.advertise<std_msgs::Int8>("Voltage", 1000);
    pub_v = this->create_publisher<std_msgs::msg::Int8>("Voltage", 1000);
    restore(); //first restore,abort current err,don't restore
    return true;
}

void MyAGV::restore()
{
    unsigned char cmd[6] = {0xfe, 0xfe, 0x01, 0x00, 0x01, 0x02};
    boost::asio::write(sp, boost::asio::buffer(cmd));
    return;
}

void MyAGV::restoreRun()
{
    int res = 0;
    std::cout << "if you want restore run,pls input 1,then press enter" << std::endl;
    while(res != 1) {
        std::cin >> res;
        //std::cout << res;
    }
    restore();
    return;
}

bool MyAGV::readSpeed()
{
    int i, length = 0, count = 0;
    unsigned char checkSum;
    unsigned char buf_header[1] = {0};
    unsigned char buf[16] = {0};

    size_t ret;
    boost::system::error_code er2;
    bool header_found = false;
    while (!header_found) {
        ++count;
        ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
        if (ret != 1) {
            continue;
        }
        if (buf_header[0] != header[0]) {
            continue;
        }
        bool header_2_found = false;
        while (!header_2_found) {
            ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
            if (ret != 1) {
                continue;
            }
            if (buf_header[0] != header[0]) {
                continue;
            }
            header_2_found = true;
        }
        header_found = true;
    }

    ret = boost::asio::read(sp, boost::asio::buffer(buf), boost::asio::transfer_at_least(4), er2); // ready break

	if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
        int wheel_num = 0;
        for (int i = 0; i < 4; ++i) {
            if (buf[i] == 1) {
                wheel_num = i+1;
                // ROS_ERROR("ERROR %d wheel current > 2000", wheel_num);
                RCLCPP_ERROR(this->get_logger(), "ERROR %d wheel current > 2000", wheel_num);
            }
        }
        restoreRun();
        return false;
    }
    if (ret != 16) {
        // ROS_ERROR("Read error");
        RCLCPP_ERROR(this->get_logger(), "Read error");
        return false;
    }

    int index = 0;
    int check = 0;
    for (int i = 0; i < 15; ++i)
        check += buf[index + i];
    if (check % 256 != buf[index + 15])
	{
		// ROS_ERROR("error 3!");
        RCLCPP_ERROR(this->get_logger(), "error 3!");	
    	return false;
	}

    vx = (static_cast<double>(buf[index]) - 128.0) * 0.01;
    vy = (static_cast<double>(buf[index + 1]) - 128.0) * 0.01;
    vtheta = (static_cast<double>(buf[index + 2]) - 128.0) * 0.01;

    ax = ((buf[index + 3] + buf[index + 4] * 256 ) - 10000) * 0.001;
    ay = ((buf[index + 5] + buf[index + 6] * 256 ) - 10000) * 0.001;
    az = ((buf[index + 7] + buf[index + 8] * 256 ) - 10000) * 0.001;

    wx = ((buf[index + 9] + buf[index + 10] * 256 ) - 10000) * 0.1;
    wy = ((buf[index + 11] + buf[index + 12] * 256 ) - 10000) * 0.1;
    wz = ((buf[index + 13] + buf[index + 14] * 256 ) - 10000) * 0.1;
    
    // currentTime = ros::Time::now();
    currentTime = rclcpp::Clock().now();

    // double dt = (currentTime - lastTime).toSec();
    double dt = (currentTime - lastTime).seconds();
    double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
    double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
    double delta_th = vtheta * dt;

    x += delta_x;
    y += delta_y;
    theta += delta_th;
    lastTime = currentTime;

    return true;
}

void MyAGV::writeSpeed(double movex, double movey, double rot)
{
    std::cout << "Move MyAGV"<< std::endl;
    if (movex == 10 && movey == 10 && rot == 10)
    {
        int buf[6] = {0xfe, 0xfe ,0x01 ,0x01 ,0x01 ,0x03};
        boost::asio::write(sp, boost::asio::buffer(buf));
        unsigned char buf_header[1] = {0};

        size_t ret;
        boost::system::error_code er2;
        bool header_found = false;
        time_t now_t = time(NULL);
        while (true) {
            
            ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
            
            if (ret != 1) {
                continue;
            }
            if (buf_header[0] != header[0]) {
                continue;
            }
            bool header_2_found = false;
            while (!header_2_found) {
                ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
                if (ret != 1) {
                    continue;
                }
                if (buf_header[0] != header[0]) {
                    continue;
                }
                header_2_found = true;
            }
            header_found = true;
            ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
            if (buf_header[0] == 0x01)
            {
                ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
                if (buf_header[0] == 0x01)
                {
                    ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
                    std_msgs::msg::Int8 msg;
                    msg.data = (int)buf_header[0] / 10;
                    // ROS_INFO("Voltage: %d", msg.data);
                    RCLCPP_INFO(this->get_logger(), "Voltage: %d", msg.data);
                    // pub_v.publish(msg);
                    pub_v->publish(msg);
                    break;
                }
            }
            if (time(NULL) - now_t > 3)
            {
                // ROS_ERROR("Get Voltage timeout");
                RCLCPP_ERROR(this->get_logger(), "Get Voltage timeout");
                break;
            }
        }
    }else{
    if (movex > 1.0) movex = 1.0;
    if (movex < -1.0) movex = -1.0;
    if (movey > 1.0) movey = 1.0;
    if (movey < -1.0) movey = -1.0;
    if (rot > 1.0) rot = 1.0;
    if (rot < -1.0) rot = -1.0;

    unsigned char x_send = static_cast<signed char>(movex * 100) + 128;
    unsigned char y_send = static_cast<signed char>(movey * 100) + 128;
    unsigned char rot_send = static_cast<signed char>(rot * 100) + 128;
    unsigned char check = x_send + y_send + rot_send;

    char buf[8] = { 0 };
    buf[0] = header[0];
    buf[1] = header[1];
    buf[2] = x_send;
    buf[3] = y_send;
    buf[4] = rot_send;
    buf[5] = check;

    boost::asio::write(sp, boost::asio::buffer(buf));}
}

bool MyAGV::execute(double linearX, double linearY, double angularZ)
{   
	std::cout << "execute: " << linearX << std::endl;
    writeSpeed(linearX, linearY, angularZ);
    readSpeed(); // easy to report error 

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    tf2::Quaternion quat_tf;
    // odom_quat = tf2_ros::createQuaternionMsgFromYaw(theta); // THETA
    quat_tf.setRPY(0, 0, theta);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat_tf);
    odom_trans.transform.translation.x = x; //X
    odom_trans.transform.translation.y = y; //Y

    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // odomBroadcaster.sendTransform(odom_trans);
    odom_broadcaster->sendTransform(odom_trans);

    nav_msgs::msg::Odometry msgl;
    msgl.header.stamp = currentTime;
    msgl.header.frame_id = "odom";

    msgl.pose.pose.position.x = x;
    msgl.pose.pose.position.y = y;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;

    msgl.child_frame_id = "base_footprint";
    msgl.twist.twist.linear.x = vx;
    msgl.twist.twist.linear.y = vy;
    msgl.twist.twist.angular.z = vtheta;
    msgl.twist.covariance = odom_twist_covariance;

    // pub.publish(msgl);
    pub_odom->publish(msgl);
}
