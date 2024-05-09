#include <myagv_estimation/odom_estimation.hpp>

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf2;
using namespace std;
using namespace rclcpp;


namespace estimation
{
  // 생성자
  OdomEstimation::OdomEstimation(const rclcpp::NodeOptions& options):
    Node("odom_estimation", options),
    node_shared_ptr_(this->shared_from_this()),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, false),
    prior_(nullptr),
    filter_(nullptr),
    filter_initialized_(false),
    odom_initialized_(false),
    imu_initialized_(false),
    vo_initialized_(false),
    gps_initialized_(false),
    output_frame_("odom_combined"),
    base_footprint_frame_("base_footprint")

  {
    // 시스템 모델 생성
    ColumnVector sysNoise_Mu(6);
    sysNoise_Mu = 0;
    SymmetricMatrix sysNoise_Cov(6);
    sysNoise_Cov = 0;
    for (unsigned int i=1; i<=6; i++) sysNoise_Cov(i,i) = pow(1000.0,2);
    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    sys_pdf_   = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
    sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

    // 오도메트리 측정 모델 생성
    ColumnVector measNoiseOdom_Mu(6); 
    measNoiseOdom_Mu = 0;
    SymmetricMatrix measNoiseOdom_Cov(6);
    measNoiseOdom_Cov = 0;
    for (unsigned int i=1; i<=6; i++) measNoiseOdom_Cov(i,i) = 1;
    Gaussian measurement_Uncertainty_Odom(measNoiseOdom_Mu, measNoiseOdom_Cov);
    Matrix Hodom(6,6);  Hodom = 0;
    Hodom(1,1) = 1;    Hodom(2,2) = 1;    Hodom(6,6) = 1;
    odom_meas_pdf_   = new LinearAnalyticConditionalGaussian(Hodom, measurement_Uncertainty_Odom);
    odom_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(odom_meas_pdf_);

    // IMU 측정 모델 생성
    ColumnVector measNoiseImu_Mu(3);  measNoiseImu_Mu = 0;
    SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
    for (unsigned int i=1; i<=3; i++) measNoiseImu_Cov(i,i) = 1;
    Gaussian measurement_Uncertainty_Imu(measNoiseImu_Mu, measNoiseImu_Cov);
    Matrix Himu(3,6);  Himu = 0;
    Himu(1,4) = 1;    Himu(2,5) = 1;    Himu(3,6) = 1;
    imu_meas_pdf_   = new LinearAnalyticConditionalGaussian(Himu, measurement_Uncertainty_Imu);
    imu_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(imu_meas_pdf_);

    // VO, Visual Odometry 측정 모델 생성
    ColumnVector measNoiseVo_Mu(6);  measNoiseVo_Mu = 0;
    SymmetricMatrix measNoiseVo_Cov(6);  measNoiseVo_Cov = 0;
    for (unsigned int i=1; i<=6; i++) measNoiseVo_Cov(i,i) = 1;
    Gaussian measurement_Uncertainty_Vo(measNoiseVo_Mu, measNoiseVo_Cov);
    Matrix Hvo(6,6);  Hvo = 0;
    Hvo(1,1) = 1;    Hvo(2,2) = 1;    Hvo(3,3) = 1;    Hvo(4,4) = 1;    Hvo(5,5) = 1;    Hvo(6,6) = 1;
    vo_meas_pdf_   = new LinearAnalyticConditionalGaussian(Hvo, measurement_Uncertainty_Vo);
    vo_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(vo_meas_pdf_);

    // GPS 측정 모델 생성
    ColumnVector measNoiseGps_Mu(3);  measNoiseGps_Mu = 0;
    SymmetricMatrix measNoiseGps_Cov(3);  measNoiseGps_Cov = 0;
    for (unsigned int i=1; i<=3; i++) measNoiseGps_Cov(i,i) = 1;
    Gaussian measurement_Uncertainty_GPS(measNoiseGps_Mu, measNoiseGps_Cov);
    Matrix Hgps(3,6);  Hgps = 0;
    Hgps(1,1) = 1;    Hgps(2,2) = 1;    Hgps(3,3) = 1;    
    gps_meas_pdf_   = new LinearAnalyticConditionalGaussian(Hgps, measurement_Uncertainty_GPS);
    gps_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(gps_meas_pdf_);
  };



  // destructor
  // 소멸자
  OdomEstimation::~OdomEstimation(){
    if (filter_) delete filter_;
    if (prior_)  delete prior_;
    delete odom_meas_model_;
    delete odom_meas_pdf_;
    delete imu_meas_model_;
    delete imu_meas_pdf_;
    delete vo_meas_model_;
    delete vo_meas_pdf_;
    delete gps_meas_model_;
    delete gps_meas_pdf_;
    delete sys_pdf_;
    delete sys_model_;
  };


  // initialize prior density of filter
  // 필터의 사전 확률 밀도 초기화
  void OdomEstimation::initialize(const Transform& prior, const Time& time)
  {
    // std::chrono::time_point 변환 예시
    auto ns = std::chrono::nanoseconds(time.nanoseconds());
    auto chrono_time = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(ns);

    // set prior of filter
    // 필터의 사전 설정
    ColumnVector prior_Mu(6); 
    decomposeTransform(prior, prior_Mu(1), prior_Mu(2), prior_Mu(3), prior_Mu(4), prior_Mu(5), prior_Mu(6));
    SymmetricMatrix prior_Cov(6); 
    for (unsigned int i=1; i<=6; i++) {
      for (unsigned int j=1; j<=6; j++){
	      prior_Cov(i,j) = (i == j) ? pow(0.001, 2) : 0;
      }
    }
    prior_  = new Gaussian(prior_Mu,prior_Cov);
    filter_ = new ExtendedKalmanFilter(prior_);

    // remember prior
    // 사전 기억
    tf2::Stamped<tf2::Transform> stamped_prior(prior, chrono_time, output_frame_, base_footprint_frame_);
    // filter_estimate_old_vec_ = prior_Mu;
    // filter_estimate_old_ = prior;
    // filter_time_old_     = time;

    // filter initialized
    // 필터 초기화
    filter_initialized_ = true;
  }


  // update filter
  bool OdomEstimation::update(bool odom_active, bool imu_active, bool gps_active, bool vo_active, const Time&  filter_time, bool& diagnostics_res)
  {
    // only update filter when it is initialized
    // 필터가 초기화되지 않았을 경우 업데이트 불가
    if (!filter_initialized_){
      RCLCPP_INFO(this->get_logger(), "Cannot update filter when filter was not initialized first.");
      return false;
    }

    // only update filter for time later than current filter time
    // 현재 필터 시간보다 나중 시간에만 업데이트
    // double dt = (filter_time - filter_time_old_).toSec();
    double dt = (filter_time - filter_time_old_).seconds();
    if (dt == 0) return false;
    if (dt <  0){
      RCLCPP_INFO(this->get_logger(), "Will not update robot pose with time %f sec in the past.", dt)
      return false;
    }
    RCLCPP_DEBUG(this->get_logger(), "Update filter at time %f with dt %f", filter_time.seconds(), dt);


    // system update filter
    // 시스템 업데이트 필터
    // --------------------
    // for now only add system noise
    // 현재는 시스템 노이즈만 추가
    ColumnVector vel_desi(2); vel_desi = 0;
    filter_->Update(sys_model_, vel_desi);

    
    // process odom measurement
    // 오도메트리 측정 처리
    RCLCPP_DEBUG(this->get_logger(), "Process odom meas");
    if (odom_active){
      // 시간 변환 추가
      tf2::TimePoint tf2_filter_time = tf2_ros::fromRclcpp(filter_time);
      if (!tf_buffer_.canTransform(base_footprint_frame_,"wheelodom", tf2_filter_time)){
        RCLCPP_ERROR(this->get_logger(), "filter time older than odom message buffer")
        return false;
      }
      tf2::Stamped<tf2::Transform> odom_meas_;
      std::string target_frame = "wheelodom";
      tf_buffer_.lookupTransform(target_frame, base_footprint_frame_, tf2_filter_time, odom_meas_);
      if (odom_initialized_){
	      // convert absolute odom measurements to relative odom measurements in horizontal plane
        // 절대 오도메트리 측정값을 상대 오도메트리 측정값으로 변환
        tf2::Quaternion q;
        q.setRPY(0, 0, filter_estimate_old_vec_(6));
	      // Transform odom_rel_frame =  Transform(tf2::createQuaternionFromYaw(filter_estimate_old_vec_(6)), filter_estimate_old_.getOrigin()) * odom_meas_old_.inverse() * odom_meas_;
        Transform odom_rel_frame =  Transform(q, filter_estimate_old_.getOrigin()) * odom_meas_old_.inverse() * odom_meas_;
	      ColumnVector odom_rel(6);
	      decomposeTransform(odom_rel_frame, odom_rel(1), odom_rel(2), odom_rel(3), odom_rel(4), odom_rel(5), odom_rel(6));
	      angleOverflowCorrect(odom_rel(6), filter_estimate_old_vec_(6));
	      // update filter
        // 필터 업데이트
	      odom_meas_pdf_->AdditiveNoiseSigmaSet(odom_covariance_ * pow(dt,2));
        RCLCPP_DEBUG(this->get_logger(), "Update filter with odom measurement %f %f %f %f %f %f", odom_rel(1), odom_rel(2), odom_rel(3), odom_rel(4), odom_rel(5), odom_rel(6));
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
    // 센서 비활성화
    else odom_initialized_ = false;

    
    // process imu measurement
    // IMU 측정 처리
    // -----------------------
    if (imu_active){
      if (!tf_buffer_.canTransform(base_footprint_frame_,"imu", filter_time, tf2::durationFromSec(0.1))){
        RCLCPP_ERROR(this->get_logger(), "filter time older than imu message buffer");
        return false;
      }
      tf2::Stamped<tf2::Transform> imu_meas_;
      try {
        imu_meas_ = tf_buffer_.lookupTransform(base_footprint_frame_, "imu", filter_time, tf2::durationFromSec(0.1));
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform imu: %s", ex.what());
        return false;
      }
      if (imu_initialized_){
	      // convert absolute imu yaw measurement to relative imu yaw measurement
        // 절대 IMU 요각 측정값을 상대 IMU 요각 측정값으로 변환
	      Transform imu_rel_frame =  filter_estimate_old_ * imu_meas_old_.inverse() * imu_meas_;
	      ColumnVector imu_rel(3); double tmp;
	      decomposeTransform(imu_rel_frame, tmp, tmp, tmp, tmp, tmp, imu_rel(3));
	      decomposeTransform(imu_meas_,     tmp, tmp, tmp, imu_rel(1), imu_rel(2), tmp);
	      angleOverflowCorrect(imu_rel(3), filter_estimate_old_vec_(6));
	      diagnostics_imu_rot_rel_ = imu_rel(3);
	      // update filter
        // 필터 업데이트
	      imu_meas_pdf_->AdditiveNoiseSigmaSet(imu_covariance_ * pow(dt,2));
	      filter_->Update(imu_meas_model_,  imu_rel);
      }
      else{
	      imu_initialized_ = true;
	      diagnostics_imu_rot_rel_ = 0;
      }
      imu_meas_old_ = imu_meas_; 
    }
    // sensor not active
    // 센서 비활성화
    else imu_initialized_ = false;
    
    
    
    // process vo measurement
    // VO, Visual Odometry 측정 처리
    // ----------------------
    if (vo_active){
      if (!tf_buffer_.canTransform(base_footprint_frame_,"vo", filter_time)){
        RCLCPP_ERROR(this->get_logger(), "filter time older than vo message buffer");
        return false;
      }
      auto vo_meas_transformed = tf_buffer_.lookupTransform(base_footprint_frame_, "vo", filter_time);
      tf2::Stamped<tf2::Transform> vo_meas_;
      // tf_buffer_.lookupTransform("vo", base_footprint_frame_, filter_time, vo_meas_);
      tf2::fromMsg(vo_meas_transformed.transform, vo_meas_);

      if (vo_initialized_){
	      // convert absolute vo measurements to relative vo measurements
        // 절대 VO 측정값을 상대 VO 측정값으로 변환
	      Transform vo_rel_frame =  filter_estimate_old_ * vo_meas_old_.inverse() * vo_meas_;
	      ColumnVector vo_rel(6);
	      decomposeTransform(vo_rel_frame, vo_rel(1),  vo_rel(2), vo_rel(3), vo_rel(4), vo_rel(5), vo_rel(6));
	      angleOverflowCorrect(vo_rel(6), filter_estimate_old_vec_(6));
	      // update filter
        // 필터 업데이트
        vo_meas_pdf_->AdditiveNoiseSigmaSet(vo_covariance_ * pow(dt,2));
        filter_->Update(vo_meas_model_,  vo_rel);
      }
      else vo_initialized_ = true;
      vo_meas_old_ = vo_meas_;
    }
    // sensor not active
    else vo_initialized_ = false;
  


    // process gps measurement
    // GPS 측정 처리
    // ----------------------
    if (gps_active){
      if (!tf_buffer_.canTransform(base_footprint_frame_,"gps", filter_time)){
        RCLCPP_ERROR(this->get_logger(), "Filter time older than gps message buffer");
        return false;
      }
      tf2::Stamped<tf2::Transform> gps_meas_; // added
      tf_buffer_.lookupTransform("gps", base_footprint_frame_, filter_time, gps_meas_);
      if (gps_initialized_){
        gps_meas_pdf_->AdditiveNoiseSigmaSet(gps_covariance_ * pow(dt,2));
        ColumnVector gps_vec(3);
        double tmp;
        //Take gps as an absolute measurement, do not convert to relative measurement
        // GPS를 절대 측정값으로 처리
        decomposeTransform(gps_meas_, gps_vec(1), gps_vec(2), gps_vec(3), tmp, tmp, tmp);
        filter_->Update(gps_meas_model_,  gps_vec);
      }
      else {
        gps_initialized_ = true;
        gps_meas_old_ = gps_meas_;
      }
    }
    // sensor not active
    // 센서 비활성화
    else gps_initialized_ = false;

  
    
    // remember last estimate
    // 마지막 추정값 기억
    filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
    // tf::Quaternion q;
    tf2::Quaternion q;
    q.setRPY(filter_estimate_old_vec_(4), filter_estimate_old_vec_(5), filter_estimate_old_vec_(6));
    filter_estimate_old_ = Transform(q,
				     Vector3(filter_estimate_old_vec_(1), filter_estimate_old_vec_(2), filter_estimate_old_vec_(3)));
    filter_time_old_ = filter_time;
    // addMeasurement(StampedTransform(filter_estimate_old_, filter_time, output_frame_, base_footprint_frame_));
    addMeasurement(tf2::Stamped<tf2::Transform>(filter_estimate_old_, filter_time, output_frame_, base_footprint_frame_));

    // diagnostics
    // 진단
    diagnostics_res = true;
    if (odom_active && imu_active){
      double diagnostics = fabs(diagnostics_odom_rot_rel_ - diagnostics_imu_rot_rel_)/dt;
      if (diagnostics > 0.3 && dt > 0.01){
	      diagnostics_res = false;
      }
    }
    return true;
  };

  void OdomEstimation::addMeasurement(const StampedTransform& meas)
  {
    RCLCPP_DEBUG(this->get_logger(), "AddMeasurement from %s to %s:  (%f, %f, %f)  (%f, %f, %f, %f)",
              meas.frame_id_.c_str(), meas.child_frame_id_.c_str(),
              meas.getOrigin().x(), meas.getOrigin().y(), meas.getOrigin().z(),
              meas.getRotation().x(),  meas.getRotation().y(), 
              meas.getRotation().z(), meas.getRotation().w());
    tf_buffer_.setTransform( meas );
  }


   //todo sukai
   // 측정값 추가 및 공분산 확인
  void OdomEstimation::addMeasurement(const tf2::Stamped<tf2::Transform>& meas, const MatrixWrapper::SymmetricMatrix& covar)
  {
    // check covariance
    // 공분산 검사
    for (unsigned int i=0; i<covar.rows(); i++){
      if (covar(i+1,i+1) == 0){
        // ROS_ERROR("Covariance specified for measurement on topic %s is zero", meas.child_frame_id_.c_str());
        RCLCPP_ERROR(this->get_logger(), "Covariance specified for measurement on topic %s is zero", meas.child_frame_id_.c_str());
        return;
      }
    }
    // add measurements
    // 측정값 추가
    addMeasurement(meas);
    if (meas.child_frame_id_ == "wheelodom") odom_covariance_ = covar;
    else if (meas.child_frame_id_ == "imu")  imu_covariance_  = covar;
    else if (meas.child_frame_id_ == "vo")   vo_covariance_   = covar;
    else if (meas.child_frame_id_ == "gps")  gps_covariance_  = covar;
    else RCLCPP_ERROR(this->get_logger(), "Adding a measurement for an unknown sensor %s", meas.child_frame_id_.c_str());
  };


  // get latest filter posterior as vector
  // 최신 필터 사후 확률을 벡터로 얻기
  void OdomEstimation::getEstimate(MatrixWrapper::ColumnVector& estimate)
  {
    estimate = filter_estimate_old_vec_;
  };

  // get filter posterior at time 'time' as Transform
  // 주어진 시간에 따른 필터 사후 확률 얻기
  void OdomEstimation::getEstimate(Time time, Transform& estimate)
  {
    tf2::Stamped<tf2::Transform> tmp;
    if (!tf_buffer_.canTransform(base_footprint_frame_,output_frame_, time)){
      // ROS_ERROR("Cannot get transform at time %f", time.toSec());
      RCLCPP_ERROR(this->get_logger(), "Cannot get transform at time %f", time.seconds());
      return;
    }
    tf_buffer_.lookupTransform(output_frame_, base_footprint_frame_, time, tmp);
    estimate = tmp;
  };

  // get filter posterior at time 'time' as Stamped Transform
  // 주어진 시간에 따른 필터 사후 확률의 스탬프된 변환 얻기
  void OdomEstimation::getEstimate(Time time, tf2::Stamped<tf2::Transform>& estimate)
  {
    if (!tf_buffer_.canTransform(output_frame_, base_footprint_frame_, time)){
      // ROS_ERROR("Cannot get transform at time %f", time.toSec());
      RCLCPP_ERROR(this->get_logger(), "Cannot get transform at time %f", time.seconds());
      return;
    }
    tf_buffer_.lookupTransform(output_frame_, base_footprint_frame_, time, estimate);
  };

  // get most recent filter posterior as PoseWithCovarianceStamped
  // 최근 필터 사후 확률을 포즈 공분산 스탬프로 얻기
  void OdomEstimation::getEstimate(geometry_msgs::msg::PoseWithCovarianceStamped& estimate)
  {
    // pose
    // StampedTransform tmp;
    tf2::Stamped<tf2::Transform> tmp;
    if (!tf_buffer_.canTransform(output_frame_, base_footprint_frame_, rclcpp::Time(0))){
      // ROS_ERROR("Cannot get transform at time %f", 0.0);
      RCLCPP_ERROR(this->get_logger(), "Cannot get transform at time %f", 0.0);
      return;
    }
    tf_buffer_.lookupTransform(output_frame_, base_footprint_frame_, rclcpp::Time(0), tmp);
    // poseTFToMsg(tmp, estimate.pose.pose);
    tf2::toMsg(tmp, estimate.pose.pose);

    // header
    estimate.header.stamp = tmp.stamp_;
    estimate.header.frame_id = output_frame_;

    // covariance
    // 공분산
    SymmetricMatrix covar =  filter_->PostGet()->CovarianceGet();
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
	estimate.pose.covariance[6*i+j] = covar(i+1,j+1);
  };

  // correct for angle overflow
  // 각도 오버플로우 보정
  void OdomEstimation::angleOverflowCorrect(double& a, double ref)
  {
    while ((a-ref) >  M_PI) a -= 2*M_PI;
    while ((a-ref) < -M_PI) a += 2*M_PI;
  };

  // decompose Transform into x,y,z,Rx,Ry,Rz
  // 변환 분해
  void OdomEstimation::decomposeTransform(const tf2::Stamped<tf2::Transform>& trans, 
					   double& x, double& y, double&z, double&Rx, double& Ry, double& Rz){
    x = trans.getOrigin().x();   
    y = trans.getOrigin().y(); 
    z = trans.getOrigin().z(); 
    trans.getBasis().getEulerYPR(Rz, Ry, Rx);
  };

  // decompose Transform into x,y,z,Rx,Ry,Rz
  // why twice
  void OdomEstimation::decomposeTransform(const Transform& trans, 
					   double& x, double& y, double&z, double&Rx, double& Ry, double& Rz){
    x = trans.getOrigin().x();   
    y = trans.getOrigin().y(); 
    z = trans.getOrigin().z(); 
    trans.getBasis().getEulerYPR(Rz, Ry, Rx);
  };

  void OdomEstimation::setOutputFrame(const std::string& output_frame){
	output_frame_ = output_frame;
  };

  void OdomEstimation::setBaseFootprintFrame(const std::string& base_frame){
	base_footprint_frame_ = base_frame;
  };

}; // namespace