#include <myagv_pose_ekf/odom_estimation.hpp>

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf2;
using namespace std;

namespace estimation
{
    OdomEstimation::OdomEstimation():
        prior_(NULL),
        filter_(NULL),
        filter_initialized_(false),
        odom_active_(false),
        imu_active_(false),
        gps_active_(false),
        vo_active_(false),
        output_frame_("odom"),
        base_footprint_frame_("base_footprint")
    {
        // System Model
        ColumnVector sysNoise_Mu(6); sysNoise_Mu = 0;
        SymmetricMatrix sysNoise_Cov(6); sysNoise_Cov = 0;
        for(unsigned int i = 1; i <= 6; i++)
        {
            sysNoise_Cov(i,i) = pow(1000, 0.2);
        }
        Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Mu);
        sys_pdf_ = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
        sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

        // create MEASUREMENT MODEL ODOM
        ColumnVector measNoiseOdom_Mu(6); measNoiseOdom_Mu = 0;
        SymmetricMatrix measNoiseOdom_Cov(6); measNoiseOdom_Cov = 0;
        for(unsigned int i = 1; i <= 6; i++)
        {
            measNoiseOdom_Cov(i,i) = 1;
        }
        Gaussian measurement_Uncertainty_Odom(measNoiseOdom_Mu, measNoiseOdom_Cov);
        Matrix Hodom(6,6); Hodom = 0;
        Hodom(1,1) = 1; Hodom(2,2) = 1; Hodom(6,6) = 1;
        odom_meas_pdf_ = new LinearAnalyticConditionalGaussian(Hodom, measurement_Uncertainty_Odom);
        odom_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(odom_meas_pdf_);

        // create MEASUREMENT MODEL IMU
        ColumnVector measNoiseImu_Mu(3); measNoiseImu_Mu = 0;
        SymmetricMatrix measNoiseImu_Cov(3); measNoiseImu_Cov = 0;
        for(unsigned int i = 1; i <= 3; i++)
        {
            measNoiseImu_Cov(i,i) = 1;
        }
        Gaussian measurement_Uncertainty_Imu(measNoiseImu_Mu, measNoiseImu_Cov);
        Matrix Himu(3,6); Himu = 0;
        Himu(1,4) = 1; Himu(2,5) = 1; Himu(3,6) = 1;
        imu_meas_pdf_ = new LinearAnalyticConditionalGaussian(Himu, measurement_Uncertainty_Imu);
        imu_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(imu_meas_pdf_);

        // create MEASUREMENT MODEL VO
        ColumnVector measNoiseVo_Mu(6); measNoiseVo_Mu = 0;
        SymmetricMatrix measNoiseVo_Cov(6); measNoiseVo_Cov = 0;
        for(unsigned int i = 1; i <= 6; i++)
        {
            measNoiseVo_Cov(i,i) = 1;
        }
        Gaussian measurement_Uncertainty_Vo(measNoiseVo_Mu, measNoiseVo_Cov);
        Matrix Hvo(6,6); Hvo = 0;
        Hvo(1,1) = 1; Hvo(2,2) = 1; Hvo(3,3) = 1, Hvo(4,4) = 1; Hvo(5,5) = 1; Hvo(6,6) = 1;
        vo_meas_pdf_ = new LinearAnalyticConditionalGaussian(Hvo, measurement_Uncertainty_Vo);
        vo_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(vo_meas_pdf_);

        // create MEASUREMENT MODEL GPS
        ColumnVector measNoiseGps_Mu(3); measNoiseGps_Mu = 0;
        SymmetricMatrix measNoiseGps_Cov(3); measNoiseGps_Cov = 0;
        for(unsigned int i = 1; i <= 3; i++)
        {
            measNoiseGps_Cov(i,i) = 1;
        }
        Gaussian measurement_Uncertainty_Gps(measNoiseGps_Mu, measNoiseGps_Cov);
        Matrix Hgps(3,6); Hgps = 0;
        Hgps(1,1) = 1; Hgps(2,2) = 1; Hgps(3,3) = 1;
        gps_meas_pdf_ = new LinearAnalyticConditionalGaussian(Hgps, measurement_Uncertainty_Gps);
        gps_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(gps_meas_pdf_);
    };

    OdomEstimation::~OdomEstimation()
    {
        if (prior_)
        {
            delete prior_;
        }
        if (filter_)
        {
            delete filter_;
        }
        delete sys_pdf_;
        delete sys_model_;
        delete odom_meas_pdf_;
        delete odom_meas_model_;
        delete imu_meas_pdf_;
        delete imu_meas_model_;
        delete vo_meas_pdf_;
        delete vo_meas_model_;
        delete gps_meas_pdf_;
        delete gps_meas_model_;
    };

    void OdomEstimation::initalize(const tf2::Transform& prior, const rclcpp::Time& time)
    {
        ColumnVector prior_Mu(6);
        decomposeTransform(prior, prior_Mu(1), prior_Mu(2), prior_Mu(3), prior_Mu(4), prior_Mu(5), prior_Mu(6));
        SymmetricMatrix prior_Cov(6);
        for(unsigned int i = 1; i <= 6; i++)
        {
            for(unsigned int j = 1; j <= 6; j++)
            {
                if (i == j)
                {
                    prior_Cov(i,j) = pow(0.001, 2);
                }
                else
                {
                    prior_Cov(i,j) = 0;
                }
            }
        }
        prior_ = new Gaussian(prior_Mu, prior_Cov);
        filter_ = new ExtendedKalmanFilter(prior_);
        
        addMeaSurement(StamedTransform(prior, prior_Cov, time, output_frame_, base_footprint_frame_));
        filter_estimate_old_vec_ = prior_Mu;
        filter_estimate_old_ = prior;
        filter_time_old_ = time;
        filter_initialized_ = true;
    }

    bool OdomEstimation::update(bool odom_active, bool imu_active, bool gps_active, bool vo_active, const rclcpp::Time& fillter_time, bool& diagnostics_res)
    {
        if (!filter_initialized_)
        {
            RCL_CPP_INFO(rclcpp::get_logger("OdomEstimation"), "Filter not initialized!");
            return false;
        }
        double dt = (fillter_time - filter_time_old_).seconds();
        if (dt == 0) return false;
        if (dt  < 0){
            RCL_CPP_INFO(rclcpp::get_logger("OdomEstimation"), "Will not update robot pose with time %f sec in the past", dt);
            return false;
        }
        RCL_CPP_DEBUG(rclcpp::get_logger("OdomEstimation"), "Update fillter at time %f with dt %f", fillter_time.seconds(), dt);

        ColumnVector vel_desi(2); vel_desi = 0;
        fillter_->Update(sys_model_, vel_desi);

        RCL_CPP_DEBUG(rclcpp::get_logger("OdomEstimation"), "Process odom meas");
        if(odom_active){
            if(!transformer_.canTransform(base_footprint_frame_, "wheelodom", fillter_time)){
                RCL_CPP_INFO(rclcpp::get_logger("OdomEstimation"), "fillter time oldet than odom message buffer");
                return false;
            }

            transformer_.lookupTransform("wheelodom", base_footprint_frame_, fillter_time, odom_meas_);
            if (odom_initialized_){
                // convert absolute odom measurements to relative odom measurements in horizontal plane
                Transform odom_rel_frame =  Transform(tf::createQuaternionFromYaw(filter_estimate_old_vec_(6)), 
                                    filter_estimate_old_.getOrigin()) * odom_meas_old_.inverse() * odom_meas_;
                ColumnVector odom_rel(6); 
                decomposeTransform(odom_rel_frame, odom_rel(1), odom_rel(2), odom_rel(3), odom_rel(4), odom_rel(5), odom_rel(6));
                angleOverflowCorrect(odom_rel(6), filter_estimate_old_vec_(6));
                // update filter
                odom_meas_pdf_->AdditiveNoiseSigmaSet(odom_covariance_ * pow(dt,2));

                    ROS_DEBUG("Update filter with odom measurement %f %f %f %f %f %f", 
                            odom_rel(1), odom_rel(2), odom_rel(3), odom_rel(4), odom_rel(5), odom_rel(6));
                filter_->Update(odom_meas_model_, odom_rel);
                diagnostics_odom_rot_rel_ = odom_rel(6);
            }
            else{
                odom_initialized_ = true;
                diagnostics_odom_rot_rel_ = 0;
            }
            odom_meas_old_ = odom_meas_;
        }
        // sensor not active
        else odom_initialized_ = false;
            
    }
}