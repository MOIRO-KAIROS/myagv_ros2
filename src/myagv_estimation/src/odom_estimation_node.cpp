#include <myagv_estimation/odom_estimation_node.hpp>

using namespace MatrixWrapper;
using namespace std;
using namespace rclcpp;
using namespace tf2;


static const double EPS = 1e-5;


//#define __EKF_DEBUG_FILE__

namespace estimation
{
  // constructor
  // 생성자
  OdomEstimationNode::OdomEstimationNode()
    : Node("odom_estimation_node") // 추가됨
      odom_active_(false),
      imu_active_(false),
      vo_active_(false),
      gps_active_(false),
      odom_initializing_(false),
      imu_initializing_(false),
      vo_initializing_(false),
      gps_initializing_(false),
      odom_covariance_(6),
      imu_covariance_(3),
      vo_covariance_(6),
      gps_covariance_(3),
      odom_callback_counter_(0),
      imu_callback_counter_(0),
      vo_callback_counter_(0),
      gps_callback_counter_(0),
      ekf_sent_counter_(0)
  {
    // ros::NodeHandle nh_private("~");
    // ros::NodeHandle nh;

    // paramters
    // nh_private.param("output_frame", output_frame_, std::string("odom_combined"));
    // nh_private.param("base_footprint_frame", base_footprint_frame_, std::string("base_footprint"));
    // nh_private.param("sensor_timeout", timeout_, 1.0);
    // nh_private.param("odom_used", odom_used_, true);
    // nh_private.param("odom_data", odom_data_, std::string("/odom_data"));//odom_data_,imu_data_
    // nh_private.param("imu_used",  imu_used_, true);
    // nh_private.param("imu_data",  imu_data_, std::string("/imu_data"));
    // nh_private.param("vo_used",   vo_used_, true);
    // nh_private.param("gps_used",   gps_used_, false);
    // nh_private.param("debug",   debug_, false);
    // nh_private.param("self_diagnose",  self_diagnose_, false);
    // double freq;
    // nh_private.param("freq", freq, 30.0);

    // 파라미터 설정
    this->declare_parameter("output_frame", "odom_combined");
    this->declare_parameter("base_footprint_frame", "base_footprint");
    this->declare_parameter("sensor_timeout", 1.0);
    this->declare_parameter("odom_used", true);
    this->declare_parameter("imu_used", true);
    this->declare_parameter("vo_used", true);
    this->declare_parameter("gps_used", false);
    this->declare_parameter("debug", false);
    this->declare_parameter("self_diagnose", false);
    this->declare_parameter("freq", 30.0);

    output_frame_ = this->get_parameter("output_frame").as_string();
    base_footprint_frame_ = this->get_parameter("base_footprint_frame").as_string();
    timeout_ = this->get_parameter("sensor_timeout").as_double();
    odom_used_ = this->get_parameter("odom_used").as_bool();
    imu_used_ = this->get_parameter("imu_used").as_bool();
    vo_used_ = this->get_parameter("vo_used").as_bool();
    gps_used_ = this->get_parameter("gps_used").as_bool();
    debug_ = this->get_parameter("debug").as_bool();
    self_diagnose_ = this->get_parameter("self_diagnose").as_bool();
    double freq = this->get_parameter("freq").as_double();

    // tf_prefix_ = tf::getPrefixParam(nh_private);
    // output_frame_ = tf::resolve(tf_prefix_, output_frame_);
    // base_footprint_frame_ = tf::resolve(tf_prefix_, base_footprint_frame_);

    // ROS_INFO_STREAM("output frame: " << output_frame_);
    // ROS_INFO_STREAM("base frame: " << base_footprint_frame_);
    RCLCPP_INFO(this->get_logger(), "Output frame: %s", output_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Base frame: %s", base_footprint_frame_.c_str());

    // set output frame and base frame names in OdomEstimation filter
    // so that user-defined tf frames are respected
    // 필터 출력 프레임과 기반 프레임 설정
    my_filter_.setOutputFrame(output_frame_);
    my_filter_.setBaseFootprintFrame(base_footprint_frame_);

    // 타이머 설정
    // timer_ = nh_private.createTimer(ros::Duration(1.0/max(freq,1.0)), &OdomEstimationNode::spin, this);
    timer_ = this->create_wall_timer(1000ms / max(freq, 1.0), std::bind(&OdomEstimationNode::spin, this));


    // pose_pub_ = nh_private.advertise<geometry_msgs::PoseWithCovarianceStamped>("odom_combined", 10);
    // pose_pub_ = nh_private.advertise<nav_msgs::Odometry>("odom_combined", 10); 이거 왜 안쓰지


    // initialize
    // filter_stamp_ = Time::now(); 왜 안씀

    // subscribe to odom messages
    if (odom_used_){
      // ROS_DEBUG("Odom sensor can be used");
      RCLCPP_DEBUG(this->get_logger(), "Odom sensor can be used");
      // odom_sub_ = nh.subscribe(odom_data_, 10, &OdomEstimationNode::odomCallback, this);
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&OdomEstimationNode::odomCallback, this, std::placeholders::_1));
    }
    else RCLCPP_DEBUG(this->get_logger(), "Odom sensor will NOT be used"); // ROS_DEBUG("Odom sensor will NOT be used");

    // subscribe to imu messages
    if (imu_used_){
      // ROS_DEBUG("Imu sensor can be used");
      RCLCPP_DEBUG(this->get_logger(), "Imu sensor can be used");
      // imu_sub_ = nh.subscribe(imu_data_, 10,  &OdomEstimationNode::imuCallback, this);
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&OdomEstimationNode::imuCallback, this, std::placeholders::_1));
    }
    else RCLCPP_DEBUG(this->get_logger(), "Imu sensor will NOT be used"); // ROS_DEBUG("Imu sensor will NOT be used");

    // subscribe to vo messages
    if (vo_used_){
      // ROS_DEBUG("VO sensor can be used");
      RCLCPP_DEBUG(this->get_logger(), "VO sensor can be used");
      // vo_sub_ = nh.subscribe("vo", 10, &OdomEstimationNode::voCallback, this);
      vo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("vo", 10, std::bind(&OdomEstimationNode::voCallback, this, std::placeholders::_1));
    }
    else RCLCPP_DEBUG(this->get_logger(), "VO sensor will NOT be used"); // ROS_DEBUG("VO sensor will NOT be used");

    if (gps_used_){
      // ROS_DEBUG("GPS sensor can be used");
      RCLCPP_DEBUG(this->get_logger(), "GPS sensor can be used");
      // gps_sub_ = nh.subscribe("gps", 10, &OdomEstimationNode::gpsCallback, this);
      gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps", 10, std::bind(&OdomEstimationNode::gpsCallback, this, std::placeholders::_1));
    
    }
    else RCLCPP_DEBUG(this->get_logger(), "GPS sensor will NOT be used"); // ROS_DEBUG("GPS sensor will NOT be used");


    // publish state service
    // state_srv_ = nh_private.advertiseService("get_status", &OdomEstimationNode::getStatus, this);
    state_srv_ = this->create_service<custom_srvs::srv::GetStatus>("get_status", std::bind(&OdomEstimationNode::getStatus, this, std::placeholders::_1, std::placeholders::_2));

    if (debug_){
      // open files for debugging
      odom_file_.open("/tmp/odom_file.txt");
      imu_file_.open("/tmp/imu_file.txt");
      vo_file_.open("/tmp/vo_file.txt");
      gps_file_.open("/tmp/gps_file.txt");
      corr_file_.open("/tmp/corr_file.txt");

  
    }
  };




  // destructor
  OdomEstimationNode::~OdomEstimationNode(){

    if (debug_){
      // close files for debugging
      odom_file_.close();
      imu_file_.close();
      gps_file_.close();
      vo_file_.close();
      corr_file_.close();
    }
  };





  //todo callback function for odom data
  void OdomEstimationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    odom_callback_counter_++;
    odomecallbackmsg=*odom;
    // ROS_DEBUG("Odom callback at time %f ", ros::Time::now().toSec());
    RCLCPP_DEBUG(this->get_logger(), "Odom callback at time %lf ", this->now().seconds());
    assert(odom_used_);

    // receive data 
    odom_stamp_ = odom->header.stamp;
    // odom_time_  = Time::now();
    odom_time_ = this->now();
    tf2::Quaternion q;
    // tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    tf2::fromMsg(odom->pose.pose.orientation, q);
    odom_meas_  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        odom_covariance_(i+1, j+1) = odom->pose.covariance[6*i+j];
    // odom_estimation.cpp 334//todo  发布odom  advertise our estimation sukai
    // 오도메트리 활성화 검사 및 초기화
    if (!odom_active_) {
      if (!odom_initializing_){
          odom_initializing_ = true;
          odom_init_stamp_ = odom_stamp_;
          // ROS_INFO("Initializing Odom sensor");
          RCLCPP_INFO(this->get_logger(), "Initializing Odom sensor");
      }
      if ( filter_stamp_ >= odom_init_stamp_){
	        odom_active_ = true;
          odom_initializing_ = false;
          // ROS_INFO("Odom sensor activated");
          RCLCPP_INFO(this->get_logger(), "Odom sensor activated");
      }
      else RCLCPP_DEBUG(this->get_logger(), "Waiting to activate Odom, because Odom measurements are still %f sec in the future.", (odom_init_stamp_ - filter_stamp_).seconds());
      // ROS_DEBUG("Waiting to activate Odom, because Odom measurements are still %f sec in the future.", (odom_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      double tmp, yaw;
      odom_meas_.getBasis().getEulerYPR(yaw, tmp, tmp);
      // odom_file_<< fixed <<setprecision(5) << ros::Time::now().toSec() << " " << odom_meas_.getOrigin().x() << " " << odom_meas_.getOrigin().y() << "  " << yaw << "  " << endl;
      odom_file_ << fixed << setprecision(5) << this->now().seconds() << " " << odom_meas_.getOrigin().x() << " " << odom_meas_.getOrigin().y() << "  " << yaw << "  " << endl;
    }
  };




  // callback function for imu data
  void OdomEstimationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu) // const ImuConstPtr& imu
  {
    imu_callback_counter_++;

    // IMU 사용 여부 확인
    assert(imu_used_);

    // receive data
    // 데이터 수신
    imu_stamp_ = imu->header.stamp;
    tf2::Quaternion orientation;
    // quaternionMsgToTF(imu->orientation, orientation);
    tf2::fromMsg(imu->orientation, orientation);
    imu_meas_ = Transform(orientation, Vector3(0,0,0));
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        imu_covariance_(i+1, j+1) = imu->orientation_covariance[3*i+j];

    // Transforms imu data to base_footprint frame
    // IMU 데이터를 base_footprint 프레임으로 변환
    // 가장 많은 변환이 있었던 코드임.

    // if (!robot_state_.waitForTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, ros::Duration(0.5))){
    // warn when imu was already activated, not when imu is not active yet
    // if (imu_active_)
    //  ROS_ERROR("Could not transform imu message from %s to %s", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
    // else if (my_filter_.isInitialized())
    //  ROS_WARN("Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
    // else 
    //   ROS_DEBUG("Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
    // return;
    // }
    // StampedTransform base_imu_offset;
    // robot_state_.lookupTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, base_imu_offset);
    // imu_meas_ = imu_meas_ * base_imu_offset;

    geometry_msgs::msg::TransformStamped base_imu_offset;
    try {
      base_imu_offset = tf_buffer_->lookupTransform(base_footprint_frame_, imu->header.frame_id, tf2::timeFromSec(imu_stamp_.seconds()), tf2::durationFromSec(0.5));
    } catch (tf2::TransformException &ex) {
      if (imu_active_)
        RCLCPP_ERROR(this->get_logger(), "Could not transform imu message from %s to %s: %s", imu->header.frame_id.c_str(), base_footprint_frame_.c_str(), ex.what());
      else if (my_filter_.isInitialized())
        RCLCPP_WARN(this->get_logger(), "Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      else 
        RCLCPP_DEBUG(this->get_logger(), "Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      return;
    }

    imu_meas_ *= Transform(base_imu_offset.transform.rotation, Vector3(base_imu_offset.transform.translation.x, base_imu_offset.transform.translation.y, base_imu_offset.transform.translation.z));

    // imu_time_  = Time::now();
    imu_time_ = this->now();

    // manually set covariance untile imu sends covariance
    if (imu_covariance_(1,1) == 0.0){
      SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
      measNoiseImu_Cov(1,1) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(2,2) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(3,3) = pow(0.00017,2);  // = 0.01 degrees / sec
      imu_covariance_ = measNoiseImu_Cov;
    }

    // my_filter_.addMeasurement(StampedTransform(imu_meas_.inverse(), imu_stamp_, base_footprint_frame_, "imu"), imu_covariance_);
    my_filter_.addMeasurement(Transform(imu_meas_.inverse()), imu_stamp_, base_footprint_frame_, "imu", imu_covariance_);
    
    // activate imu
    // IMU 활성화
    if (!imu_active_) {
      if (!imu_initializing_){
	      imu_initializing_ = true;
	      imu_init_stamp_ = imu_stamp_;
	      // ROS_INFO("Initializing Imu sensor");
        RCLCPP_INFO(this->get_logger(), "Initializing Imu sensor");
      }
      if ( filter_stamp_ >= imu_init_stamp_){
	      imu_active_ = true;
	      imu_initializing_ = false;
	      // ROS_INFO("Imu sensor activated");
        RCLCPP_INFO(this->get_logger(), "Imu sensor activated");
      }
      else RCLCPP_DEBUG(this->get_logger(), "Waiting to activate IMU, because IMU measurements are still %f sec in the future.", (imu_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      double tmp, yaw;
      imu_meas_.getBasis().getEulerYPR(yaw, tmp, tmp); 
      // imu_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< yaw << endl;
      imu_file_ << fixed << setprecision(5) << this->now().seconds() << " " << yaw << endl;
    }
  };


  // callback function for VO data
  void OdomEstimationNode::voCallback(const nav_msgs::msg::Odometry::SharedPtr vo) //const VoConstPtr& vo
  {
    vo_callback_counter_++;

    // VO 센서 사용 여부 확인
    assert(vo_used_);

    // get data
    // 데이터 수신
    vo_stamp_ = vo->header.stamp;
    // vo_time_  = Time::now();
    vo_time_ = this->now();
    // poseMsgToTF(vo->pose.pose, vo_meas_);
    tf2::convert(vo->pose.pose, vo_meas_);
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        vo_covariance_(i+1, j+1) = vo->pose.covariance[6*i+j];
    // my_filter_.addMeasurement(StampedTransform(vo_meas_.inverse(), vo_stamp_, base_footprint_frame_, "vo"), vo_covariance_);
    my_filter_.addMeasurement(Transform(vo_meas_.inverse()), vo_stamp_, base_footprint_frame_, "vo", vo_covariance_);
    
    // activate vo
    // VO, Visual Odometry 활성화
    if (!vo_active_) {
      if (!vo_initializing_){
	      vo_initializing_ = true;
	      vo_init_stamp_ = vo_stamp_;
	      // ROS_INFO("Initializing Vo sensor");
        RCLCPP_INFO(this->get_logger(), "Initializing Vo sensor");     
      }
      if (filter_stamp_ >= vo_init_stamp_){
	      vo_active_ = true;
	      vo_initializing_ = false;
	      // ROS_INFO("Vo sensor activated");
        RCLCPP_INFO(this->get_logger(), "Vo sensor activated");      
      }
      else RCLCPP_DEBUG(this->get_logger(), "Waiting to activate VO, because VO measurements are still %f sec in the future.", (vo_init_stamp_ - filter_stamp_).seconds());
    }
    
    if (debug_){
      // write to file
      // 파일에 기록
      double Rx, Ry, Rz;
      // vo_meas_.getBasis().getEulerYPR(Rz, Ry, Rx);
      tf2::Matrix3x3(vo_meas_.getRotation()).getEulerYPR(Rz, Ry, Rx);
      // vo_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< vo_meas_.getOrigin().x() << " " << vo_meas_.getOrigin().y() << " " << vo_meas_.getOrigin().z() << " "
      //         << Rx << " " << Ry << " " << Rz << endl;
      vo_file_ << fixed << setprecision(5) << this->now().seconds() << " " << vo_meas_.getOrigin().x() << " " << vo_meas_.getOrigin().y() << " " << vo_meas_.getOrigin().z() << " " << Rx << " " << Ry << " " << Rz << endl;
    }
  };


  void OdomEstimationNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps) // const GpsConstPtr& gps
  {
    gps_callback_counter_++;

    // GPS 센서 사용 여부 확인
    assert(gps_used_);

    // get data
    // 데이터 수신
    gps_stamp_ = gps->header.stamp;
    // gps_time_  = Time::now();
    gps_time_ = this->now();

    // GPS 데이터를 기하학적 변환으로 사용할 수 있게 변경
    geometry_msgs::msg::PoseWithCovariance gps_pose;
    gps_pose.pose.position.x = gps->latitude;
    gps_pose.pose.position.y = gps->longitude;
    gps_pose.pose.position.z = std::isnan(gps->altitude) ? 0.0 : gps->altitude; // GPS 고도값이 NaN인 경우 0으로 설정

    // 고도가 NaN일 경우 고도의 공분산을 무한대로 설정하여 무시하도록 함
    // if we have no linear z component in the GPS message, set it to 0 so that we can still get a transform via `tf
    // (which does not like "NaN" values)
    if (std::isnan(gps->altitude)) {
      gps_pose.covariance[6*2 + 2] = std::numeric_limits<double>::max();
    }

    // tf2를 이용하여 Pose를 Transform으로 변환
    tf2::Transform gps_meas;
    // poseMsgToTF(gps_pose.pose, gps_meas_);
    tf2::fromMsg(gps_pose.pose, gps_meas);
  
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        gps_covariance_(i+1, j+1) = gps_pose.covariance[6*i+j];
    
    // 필터에 GPS 측정값 추가
    // my_filter_.addMeasurement(StampedTransform(gps_meas_.inverse(), gps_stamp_, base_footprint_frame_, "gps"), gps_covariance_);
    my_filter_.addMeasurement(gps_meas.inverse(), gps_stamp_, base_footprint_frame_, "gps", gps_covariance_);
    
    // activate gps
    // GPS 활성화
    if (!gps_active_) {
      if (!gps_initializing_){
	      gps_initializing_ = true;
	      gps_init_stamp_ = gps_stamp_;
	      // ROS_INFO("Initializing GPS sensor");
        RCLCPP_INFO(this->get_logger(), "Initializing GPS sensor");
      }
      if (filter_stamp_ >= gps_init_stamp_){
	      gps_active_ = true;
        gps_initializing_ = false;
        RCLCPP_INFO(this->get_logger(), "GPS sensor activated");
      } else RCLCPP_DEBUG(this->get_logger(), "Waiting to activate GPS, because GPS measurements are still %f sec in the future.", (gps_init_stamp_ - filter_stamp_).seconds());
    }
  };



  // filter loop
  void OdomEstimationNode::spin()
  {
    // ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());
    RCLCPP_DEBUG(this->get_logger(), "Spin function at time %f", this->now().seconds());

    // check for timing problems
    // 시간 문제 확인
    if ( (odom_initializing_ || odom_active_) && (imu_initializing_ || imu_active_) ){
      // double diff = fabs( Duration(odom_stamp_ - imu_stamp_).toSec() );
      double diff = fabs((odom_stamp_ - imu_stamp_).seconds());
      if (diff > 1.0) RCLCPP_ERROR(this->get_logger(), "Timestamps of odometry and imu are %f seconds apart.", diff);
    }
    
    // initial value for filter stamp; keep this stamp when no sensors are active
    // 필터 타임스탬프 초기값 설정; 센서가 비활성화되면 이 타임스탬프 유지
    filter_stamp_ = this->now();
    
    // check which sensors are still active
    // 센서 활성 상태 확인
    if ((odom_active_ || odom_initializing_) && (this->now() - odom_time_).seconds() > timeout_) {
      odom_active_ = false; odom_initializing_ = false;
      // ROS_INFO("Odom sensor not active any more");
      RCLCPP_INFO(this->get_logger(), "Odom sensor not active any more");
    }
    if ((imu_active_ || imu_initializing_) && (this->now() - imu_time_).seconds() > timeout_) {
      imu_active_ = false;  imu_initializing_ = false;
      RCLCPP_INFO(this->get_logger(), "Imu sensor not active any more");
    }
    if ((vo_active_ || vo_initializing_) && (this->now() - vo_time_).seconds() > timeout_) {
      vo_active_ = false;  vo_initializing_ = false;
      RCLCPP_INFO(this->get_logger(), "VO sensor not active any more");
    }

    if ((gps_active_ || gps_initializing_) && (this->now() - gps_time_).seconds() > timeout_) {
      gps_active_ = false;  gps_initializing_ = false;
      RCLCPP_INFO(this->get_logger(), "GPS sensor not active any more");
    }

    
    // only update filter when one of the sensors is active
    // 필터 업데이트는 하나 이상의 센서가 활성화되어 있을 때만 실행
    if (odom_active_ || imu_active_ || vo_active_ || gps_active_){
      
      // update filter at time where all sensor measurements are available
      // 모든 센서 측정값이 가능한 시간에 필터 업데이트

      // if (odom_active_)  filter_stamp_ = min(filter_stamp_, odom_stamp_);
      // if (imu_active_)   filter_stamp_ = min(filter_stamp_, imu_stamp_);
      // if (vo_active_)    filter_stamp_ = min(filter_stamp_, vo_stamp_);
      // if (gps_active_)  filter_stamp_ = min(filter_stamp_, gps_stamp_);

      std::vector<rclcpp::Time> times = {odom_stamp_, imu_stamp_, vo_stamp_, gps_stamp_};
      filter_stamp_ = *std::min_element(times.begin(), times.end());

      
      // update filter
      // 필터 업데이트
      if ( my_filter_.isInitialized() )  {
        bool diagnostics = true;
        if (my_filter_.update(odom_active_, imu_active_,gps_active_, vo_active_,  filter_stamp_, diagnostics)){
          
          // output most recent estimate and relative covariance
          // 가장 최근의 추정치 및 상대 공분산 출력
          geometry_msgs::msg::PoseWithCovarianceStamped output;
          my_filter_.getEstimate(output_);
          //todo sukai
          nav_msgs::Odometry odomMsgs;

          odomMsgs.header.frame_id=output_.header.frame_id;
          odomMsgs.header.seq=output_.header.seq;
          odomMsgs.header.stamp=Time::now();
          for (unsigned int i=0; i<6; i++)
              for (unsigned int j=0; j<6; j++)
                  odomMsgs.pose.covariance[6*i+j] = output_.pose.covariance[6*i+j];
          /**
          odomMsgs.pose.pose.position.x=output_.pose.pose.position.x;
          odomMsgs.pose.pose.position.y=output_.pose.pose.position.y;
          odomMsgs.pose.pose.position.z=output_.pose.pose.position.z;
          odomMsgs.pose.pose.orientation.x=output_.pose.pose.orientation.x;
          odomMsgs.pose.pose.orientation.y=output_.pose.pose.orientation.y;
          odomMsgs.pose.pose.orientation.z=output_.pose.pose.orientation.z;
          odomMsgs.pose.pose.orientation.w=output_.pose.pose.orientation.w;
          odomMsgs.twist.twist.linear.x=odomecallbackmsg.twist.twist.linear.x;
          odomMsgs.twist.twist.linear.y=odomecallbackmsg.twist.twist.linear.y;
          odomMsgs.twist.twist.linear.z=odomecallbackmsg.twist.twist.linear.z;
          odomMsgs.twist.twist.angular.x=odomecallbackmsg.twist.twist.angular.x;
          odomMsgs.twist.twist.angular.y=odomecallbackmsg.twist.twist.angular.y;
          odomMsgs.twist.twist.angular.z=odomecallbackmsg.twist.twist.angular.z;
          for (unsigned int i=0; i<6; i++)
              for (unsigned int j=0; j<6; j++)
                  odomMsgs.twist.covariance[6*i+j] = odomecallbackmsg.twist.covariance[6*i+j];
          */

          odomMsgs.pose.pose = output_.pose.pose;
          // pose_pub_.publish(odomMsgs);
          pose_pub_->publish(odomMsgs);

          // pose_pub_.publish(output_);
          ekf_sent_counter_++;
          
          // broadcast most recent estimate to TransformArray
          // 최신 추정치를 TransformArray로 방송
          // StampedTransform tmp;
          geometry_msgs::msg::TransformStamped tmp;
          // my_filter_.getEstimate(ros::Time(), tmp);
          my_filter_.getEstimate(tmp);
          
          if(!vo_active_ && !gps_active_)
            // tmp.getOrigin().setZ(0.0);
            tmp.transform.translation.z = 0.0;
          // odom_broadcaster_.sendTransform(StampedTransform(tmp, tmp.stamp_, output_frame_, base_footprint_frame_));
          tf_broadcaster_.sendTransform(tmp);
          
          if (debug_){
            // write to file
            // 디버그 모드에서 파일에 기록
            ColumnVector estimate; 
            my_filter_.getEstimate(estimate);
            // corr_file_ << fixed << setprecision(5)<<ros::Time::now().toSec()<<" ";
            corr_file_ << fixed << setprecision(5) << this->now().seconds() << " ";
            
            for (unsigned int i=1; i<=6; i++)
             corr_file_ << estimate(i) << " ";
            corr_file_ << endl;
          }
        }
        if (self_diagnose_ && !diagnostics)
          // ROS_WARN("Robot pose ekf diagnostics discovered a potential problem");
          RCLCPP_WARN(this->get_logger(), "Robot pose ekf diagnostics discovered a potential problem");
      }


      // initialize filer with odometry frame
      if (imu_active_ && gps_active_ && !my_filter_.isInitialized()) {
	      // Quaternion q = imu_meas_.getRotation();
        // Vector3 p = gps_meas_.getOrigin();
        // Transform init_meas_ = Transform(q, p);
        Transform init_meas_ = Transform(Quaternion(imu_meas_.rotation), Vector3(gps_meas_.translation));
        my_filter_.initialize(init_meas_, gps_stamp_);
        // ROS_INFO("Kalman filter initialized with gps and imu measurement");
        RCLCPP_INFO(this->get_logger(), "Kalman filter initialized with gps and imu measurement");
      }	
      else if ( odom_active_ && gps_active_ && !my_filter_.isInitialized()) {
	      // Quaternion q = odom_meas_.getRotation();
        // Vector3 p = gps_meas_.getOrigin();
        // Transform init_meas_ = Transform(q, p);
        Transform init_meas_ = Transform(Quaternion(odom_meas_.rotation), Vector3(gps_meas_.translation));
        my_filter_.initialize(init_meas_, gps_stamp_);
        // ROS_INFO("Kalman filter initialized with gps and odometry measurement");
        RCLCPP_INFO(this->get_logger(), "Kalman filter initialized with gps and odometry measurement");
      }
      else if ( vo_active_ && gps_active_ && !my_filter_.isInitialized()) {
	      // Quaternion q = vo_meas_.getRotation();
        // Vector3 p = gps_meas_.getOrigin();
        // Transform init_meas_ = Transform(q, p);
        Transform init_meas_ = Transform(Quaternion(vo_meas_.rotation), Vector3(gps_meas_.translation));
        my_filter_.initialize(init_meas_, gps_stamp_);
        // ROS_INFO("Kalman filter initialized with gps and visual odometry measurement");
        RCLCPP_INFO(this->get_logger(), "Kalman filter initialized with gps and visual odometry measurement");
      }
      else if ( odom_active_  && !gps_used_ && !my_filter_.isInitialized()){
        my_filter_.initialize(odom_meas_, odom_stamp_);
        // ROS_INFO("Kalman filter initialized with odom measurement");
        RCLCPP_INFO(this->get_logger(), "Kalman filter initialized with odom measurement");
      }
      else if ( vo_active_ && !gps_used_ && !my_filter_.isInitialized()){
        my_filter_.initialize(vo_meas_, vo_stamp_);
        // ROS_INFO("Kalman filter initialized with vo measurement");
        RCLCPP_INFO(this->get_logger(), "Kalman filter initialized with vo measurement");
      }
    }
  };


// bool OdomEstimationNode::getStatus(robot_pose_ekf::GetStatus::Request& req, robot_pose_ekf::GetStatus::Response& resp)
bool OdomEstimationNode::getStatus(const std::shared_ptr<rmw_request_id_t> request, const std::shared_ptr<myagv_estimation::srv::GetStatus::Request> req, std::shared_ptr<myagv_estimation::srv::GetStatus::Response> resp)
{
  stringstream ss;
  ss << "Input:" << endl;
  ss << " * Odometry sensor" << endl;
  ss << "   - is "; if (!odom_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!odom_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << odom_callback_counter_ << " messages" << endl;
  // ss << "   - listens to topic " << odom_sub_.getTopic() << endl;
  ss << "   - listens to topic " << odom_sub_->get_topic_name() << std::endl;
  ss << " * IMU sensor" << endl;
  ss << "   - is "; if (!imu_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!imu_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << imu_callback_counter_ << " messages" << endl;
  // ss << "   - listens to topic " << imu_sub_.getTopic() << endl;
  ss << "   - listens to topic " << vo_sub_->get_topic_name() << std::endl;
  ss << " * Visual Odometry sensor" << endl;
  ss << "   - is "; if (!vo_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!vo_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << vo_callback_counter_ << " messages" << endl;
  // ss << "   - listens to topic " << vo_sub_.getTopic() << endl;
  ss << "   - listens to topic " << gps_sub_->get_topic_name() << std::endl;
  ss << " * GPS sensor" << endl;
  ss << "   - is "; if (!gps_used_) ss << "NOT "; ss << "used" << endl;
  ss << "   - is "; if (!gps_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << gps_callback_counter_ << " messages" << endl;
  // ss << "   - listens to topic " << gps_sub_.getTopic() << endl;
  ss << "   - listens to topic " << gps_sub_->get_topic_name() << std::endl;
  ss << "Output:" << endl;
  ss << " * Robot pose ekf filter" << endl;
  ss << "   - is "; if (!my_filter_.isInitialized()) ss << "NOT "; ss << "active" << endl;
  ss << "   - sent " << ekf_sent_counter_ << " messages" << endl;
  // ss << "   - pulishes on topics " << pose_pub_.getTopic() << " and /tf" << endl;
  ss << "   - publishes on topics " << pose_pub_->get_topic_name() << " and /tf" << std::endl;
  // resp.status = ss.str();
  resp->status = ss.str();
  return true;
}

}; // namespace


// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  // ros::init(argc, argv, "robot_pose_ekf");
  rclcpp::init(argc, argv);

  // create filter class
  // OdomEstimationNode my_filter_node;
  auto node = std::make_shared<OdomEstimationNode>("odom_estimation_node");

  // ros::spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}