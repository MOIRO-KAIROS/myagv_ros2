#ifndef __ODOM_ESTIMATION__HPP
#define __ODOM_ESTIMATION__HPP

// Bayesian filtering
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussianodo.hpp"

// TF2
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2_geometry_msgs/tf2_geometry_msgs.h
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// msgs
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// log files
#include <fstream>

namespace estimation
{

class OdomEstimation : public rclcpp::Node
{
public:
  /// Constructor
  OdomEstimation(const rclcpp::NodeOptions& options);

  /// Destructor
  virtual ~OdomEstimation();

  /** Update the extended Kalman filter
   * \param odom_active specifies if the odometry sensor is active or not
   * \param imu_active specifies if the imu sensor is active or not
   * \param gps_active specifies if the gps sensor is active or not
   * \param vo_active specifies if the vo sensor is active or not
   * \param filter_time update the EKF up to this time
   * \param diagnostics_res returns false if the diagnostics found that the sensor measurements are inconsistent
   * returns true on successful update
   */
  bool update(bool odom_active, bool imu_active, bool gps_active, bool vo_active, const rclcpp::Time& filter_time, bool& diagnostics_res);

  /** Initialize the extended Kalman filter
   * \param prior the prior robot pose
   * \param time the initial time of the EKF
   */
  void initialize(const tf2::Transform& prior, const rclcpp::Time& time);

  /** Check if the filter is initialized
   * returns true if the EKF has been initialized already
   */
  bool isInitialized() {return filter_initialized_;};

  /** Get the filter posterior
   * \param estimate the filter posterior as a column vector
   */
  void getEstimate(MatrixWrapper::ColumnVector& estimate);

  /** Get the filter posterior
   * \param time the time of the filter posterior
   * \param estimate the filter posterior as a tf2 transform
   */
  void getEstimate(rclcpp::Time time, tf2::Transform& estimate);

  /** Get the filter posterior
   * \param time the time of the filter posterior
   * \param estimate the filter posterior as a stamped tf2 transform
   */
  void getEstimate(rclcpp::Time time, tf2::Stamped<tf2::Transform>& estimate);

  /** Get the filter posterior
   * \param estimate the filter posterior as a pose with covariance
   */
  void getEstimate(geometry_msgs::msg::PoseWithCovarianceStamped& estimate);

  /** Add a sensor measurement to the measurement buffer
   * \param meas the measurement to add
   */
  void addMeasurement(const tf2::Stamped<tf2::Transform>& meas);

  /** Add a sensor measurement to the measurement buffer
   * \param meas the measurement to add
   * \param covar the 6x6 covariance matrix of this measurement, as defined in the PoseWithCovariance message
   */
  void addMeasurement(const tf2::Stamped<tf2::Transform>& meas, const MatrixWrapper::SymmetricMatrix& covar);

  /** Set the output frame used by tf2
   * \param output_frame the desired output frame published on /tf (e.g., odom_combined)
   */
  void setOutputFrame(const std::string& output_frame);

  /** Set the base_footprint frame of the robot used by tf2
   * \param base_frame the desired base frame from which to transform when publishing the combined odometry frame (e.g., base_footprint)
   */
  void setBaseFootprintFrame(const std::string& base_frame);

private:
  rclcpp::Node::SharedPtr node_shared_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  BFL::Gaussian* prior_ = nullptr;
  BFL::ExtendedKalmanFilter* filter_ = nullptr;
  bool filter_initialized_ = false;
  bool odom_initialized_ = false;
  bool imu_initialized_ = false;
  bool vo_initialized_ = false;
  bool gps_initialized_ = false;

  /// Correct for angle overflow
  void angleOverflowCorrect(double& a, double ref);

  // Decompose Transform into x,y,z,Rx,Ry,Rz
  void decomposeTransform(const tf2::Transform& trans,
			  double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);

  // PDF / model / filter
  BFL::AnalyticSystemModelGaussianUncertainty*            sys_model_;
  BFL::NonLinearAnalyticConditionalGaussianOdo*           sys_pdf_;
  BFL::LinearAnalyticConditionalGaussian*                 odom_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* odom_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 imu_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* imu_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 vo_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* vo_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 gps_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* gps_meas_model_;
  // BFL::Gaussian*                                          prior_;
  // BFL::ExtendedKalmanFilter*                              filter_;
  MatrixWrapper::SymmetricMatrix                          odom_covariance_, imu_covariance_, vo_covariance_, gps_covariance_;

  // Vars
  MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
  tf2::Transform filter_estimate_old_;
  tf2::Stamped<tf2::Transform> odom_meas_, odom_meas_old_, imu_meas_, imu_meas_old_, vo_meas_, vo_meas_old_, gps_meas_, gps_meas_old_;
  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener tf_listener_{tf_buffer_};

  // diagnostics
  double diagnostics_odom_rot_rel_, diagnostics_imu_rot_rel_;
  
  std::string output_frame_;
  std::string base_footprint_frame_;
  rclcpp::Time filter_time_old_; 

}; // class

}; // namespace

#endif
