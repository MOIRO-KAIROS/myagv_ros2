#ifndef __ODOM_ESTIMATION_HPP__
#define __ODOM_ESTIMATION_HPP__

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussianodo.hpp"

// TF
#include <tf2/Tf2.h>
#include <tf2/Stamped.h>
#include <tf2_ros/transform_broadcast.h>

// msgs
#include <geometry_msgs/msg/TransformStamped.h>
#include <geometry_msgs/msg/PoseWithCovarianceStamped.h>

// log files
#include <fstream>

namespace estimation
{
class OdomEstimation
{
public:
    OdomEstimation();

    virtual ~OdomEstimation();

    bool update(bool odom_active, bool imu_active, bool gps_active, bool vo_active, const rclcpp::Time& fillter_time, bool& diagnostics_res);

    void initalize(const tf2::Tf2::Transform& prior, const rclcpp::Time& time);

    bool isInitialized() {return filter_initialized_;};

    void getEstimate(MatrixWrapper::ColumnVector& estimate);

    void getEstimate(rclcpp::Time time, tf2::Tf2::Transform& estimate);

    void getEstimate(rclcpp::Time time, geometry_msgs::msg::TransformStamped& estimate);

    void getEstimate(rclcpp::Time time, geometry_msgs::msg::PoseWithCovarianceStamped& estimate);

    void addMeasurement(const tf2::Stamped<tf2::Tf2::Transform>& meas);

    void addMeasurement(const tf2::Stamped<tf2::Tf2::Transform>& meas, const MatrixWrapper::SymmetricMatrix& covar);

    void setOutputFrame(const std::string& output_frame);

    void setBaseFootprintFrame(const std::string& base_frame);

private:
    void angleOverflowCorrect(double& angle, double ref);

    void decomposeTransform(const tf2::Stamped<tf2::Tf2::Transform>& trans,
                            double& x, double& y, double& z, double& roll, double& pitch, double& yaw);

    void decomposeTransform(const tf2::Tf2::Transform& trans,
                            double& x, double& y, double& z, double& roll, double& pitch, double& yaw);

    BFL::AnalyticSystemModelGaussianUncertainty* sys_model_;
    BFL::NonLinearAnalyticConditionalGaussianOdo* sys_pdf_;
    BFL::LinearAnalyticConditionalGaussian* odom_meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *odom_meas_model_;
    BFL::AnalyticConditionalGaussian *imu_meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *imu_meas_model_;
    BFL::LinearAnalyticConditionalGaussian *vo_meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *vo_meas_model_;
    BFL::LinearAnalyticConditionalGaussian *gps_meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *gps_meas_model_;
    BFL::Gaussian* prior_;
    BFL::ExtendedKalmanFilter* filter_;
    MatrixWrapper::SymmetricMatrix odom_covariance_, imu_covariance_, gps_covariance_, vo_covariance_;

    MatrixWrapper::ColumnVector vel_desi_, fillter_estimate_old_vec_;
    tf2::Tf2::Transform fillter_estimate_old_;
    tf2::Stamped<tf2::Tf2::Transform> odom_meas_, odom_meas_old_, imu_meas_, imu_meas_old_, gps_meas_, gps_meas_old_, vo_meas_, vo_meas_old_;
    rclcpp::Time filter_time_old_l
    bool filter_initialized_, odom_initialized_, imu_initialized_, gps_initialized_, vo_initialized_;

    double diagnostic_odom_rot_rel_, diagnostic_imu_rot_rel;

    tf2_ros::TransformBroadcaster tfb_;

    string output_frame_, base_footprint_frame_; 
};

};

#endif // __ODOM_ESTIMATION_HPP__