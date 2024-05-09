#ifndef __ODOM_ESTIMATION_NODE__HPP
#define __ODOM_ESTIMATION_NODE__HPP

// ROS2 관련 헤더
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include "odom_estimation.hpp"

// 메시지 타입들
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <mutex>

// 로그 파일
#include <fstream>

namespace estimation
{
class OdomEstimationNode : public rclcpp::Node
{
public:
  /// 생성자
  OdomEstimationNode();

  /// 소멸자
  virtual ~OdomEstimationNode();

private:
  /// 주기적으로 호출될 필터 루프
  void spin();

  /// 오도메트리 데이터 콜백 함수
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  /// IMU 데이터 콜백 함수
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu);

  /// 시각 오도메트리 데이터 콜백 함수
  void voCallback(const nav_msgs::msg::Odometry::SharedPtr vo);

  /// GPS 데이터 콜백 함수
  void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr gps);

  /// 필터 상태를 확인하는 서비스
  bool getStatus(const std::shared_ptr<rmw_request_id_t> request, const std::shared_ptr<robot_pose_ekf::srv::GetStatus::Request> req, std::shared_ptr<robot_pose_ekf::srv::GetStatus::Response> resp);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, vo_sub_, gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<robot_pose_ekf::srv::GetStatus>::SharedPtr state_srv_;

  // EKF 필터 인스턴스
  OdomEstimation my_filter_;

  // 전송할 로봇 포즈 메시지
  geometry_msgs::msg::PoseWithCovarianceStamped output_;

  // 로봇 상태 관리를 위한 tf2 리스너와 브로드캐스터
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // 타임스탬프 관리
  rclcpp::Time odom_time_, imu_time_, vo_time_, gps_time_;
  bool odom_active_, imu_active_, vo_active_, gps_active_;
  bool odom_used_, imu_used_, vo_used_, gps_used_;
  double timeout_;

  // 공분산 행렬
  MatrixWrapper::SymmetricMatrix odom_covariance_, imu_covariance_, vo_covariance_, gps_covariance_;

  // 디버깅 옵션
  bool debug_, self_diagnose_;
  std::string output_frame_, base_footprint_frame_;

  // 로깅을 위한 파일 스트림
  std::ofstream odom_file_, imu_file_, vo_file_, gps_file_;

  // 카운터
  unsigned int odom_callback_counter_, imu_callback_counter_, vo_callback_counter_, gps_callback_counter_, ekf_sent_counter_;

}; // class

}; // namespace

#endif
