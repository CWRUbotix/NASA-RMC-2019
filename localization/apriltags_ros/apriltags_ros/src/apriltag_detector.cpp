#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <hci/motorCommand.h>
#include <hci/sensorValue.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/Localization.h>
#include <geometry_msgs/Pose2D.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>

//#include <cmath>
#include <math.h>  
//#include <Rot3d.h>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0 * PI;
bool found_tag_left;
bool found_tag_right;
float lookie_motor_left;// = 0.0f;
float lookie_motor_right;// = 0.0f;
float lookie_angle_right;
float lookie_angle_left;
bool lookie_swipe_left;// = true;
bool lookie_swipe_right;// = true;
bool lookie_start_pos_l;// = true;
bool lookie_start_pos_r;// = true;

void sensorCallback(const hci::sensorValue& msg) {
  //port side lookie
  if (msg.sensorID == 7) {
    lookie_angle_left = msg.value;
  }
  //starboard side lookie
  if (msg.sensorID == 8) {
    lookie_angle_right = msg.value;
  }
}


namespace apriltags_ros{

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& nh1, ros::NodeHandle& pnh): it_(nh), it_1(nh1){
  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
    ROS_WARN("No april tags specified");
  }
  else{
    try{
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
    }
  }

  if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)){
    sensor_frame_id_ = "";
  }

  std::string tag_family;
  pnh.param<std::string>("tag_family", tag_family, "36h11");

  pnh.param<bool>("projected_optics", projected_optics_, false);

  const AprilTags::TagCodes* tag_codes;
  if(tag_family == "16h5"){
    tag_codes = &AprilTags::tagCodes16h5;
  }
  else if(tag_family == "25h7"){
    tag_codes = &AprilTags::tagCodes25h7;
  }
  else if(tag_family == "25h9"){
    tag_codes = &AprilTags::tagCodes25h9;
  }
  else if(tag_family == "36h9"){
    tag_codes = &AprilTags::tagCodes36h9;
  }
  else if(tag_family == "36h11"){
    tag_codes = &AprilTags::tagCodes36h11;
  }
  else{
    ROS_WARN("Invalid tag family specified; defaulting to 36h11");
    tag_codes = &AprilTags::tagCodes36h11;
  }

  lookie_motor_left = 0.0f;
  lookie_motor_right = 0.0f;
  lookie_swipe_left = true;
  lookie_swipe_right = true;
  lookie_start_pos_l = true;
  lookie_start_pos_r = true;
  
  /*hci::motorCommand lookie_start_left;
  lookie_start_left.motorID = 6;
  lookie_start_left.value = lookie_motor_left;
  motor_commands_.publish(lookie_start_left);
  hci::motorCommand lookie_start_right;
  lookie_start_right.motorID = 7;
  lookie_start_right.value = lookie_motor_right;
  motor_commands_.publish(lookie_start_right);*/

  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
  image_sub_ = it_.subscribeCamera("image_rect", 1, &AprilTagDetector::imageCb, this);
  image_sub_1 = it_1.subscribeCamera("image_rect_1", 1, &AprilTagDetector::imageCb_1, this);
  image_sub_2 = it_1.subscribeCamera("image_rect_2", 1, &AprilTagDetector::imageCb_2, this);
  image_pub_ = it_.advertise("tag_detections_image", 1);
  detections_pub_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("tag_detections_pose", 1);
  localization_pub_ = nh.advertise<geometry_msgs::Pose2D>("localization_data", 1);
  localization_1_pub_ = nh.advertise<geometry_msgs::Pose2D>("localization_data_1", 1);
  localization_2_pub_ = nh.advertise<geometry_msgs::Pose2D>("localization_data_2", 1);
  motor_commands_ = nh.advertise<hci::motorCommand>("motorCommand", 100);
  sensor_value_sub_ = nh.subscribe("sensorValue", 100, sensorCallback);
}



void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch,
    double& roll) {

  if (atan2((double) wRo(1, 0), (double) wRo(0, 0)) > 0.) {
    yaw = fmod( (atan2((double) wRo(1, 0), (double) wRo(0, 0))) + PI, TWOPI) - PI;
  }
  else {
    yaw = fmod( (atan2((double) wRo(1, 0), (double) wRo(0, 0))) - PI, -TWOPI) + PI;
  }

  //yaw = standardRad(atan2((double) wRo(1, 0), (double) wRo(0, 0)));
  double c = cos(yaw);
  double s = sin(yaw);

  if (atan2((double) -wRo(2, 0),
          (double) (wRo(0, 0) * c + wRo(1, 0) * s)) >= 0.) {
    pitch = fmod((atan2((double) -wRo(2, 0),
          (double) (wRo(0, 0) * c + wRo(1, 0) * s))) + PI, TWOPI) - PI;
  } else {
    pitch = fmod((atan2((double) -wRo(2, 0),
          (double) (wRo(0, 0) * c + wRo(1, 0) * s))) - PI, -TWOPI) + PI;
  }
  /*pitch = standardRad(
      atan2((double) -wRo(2, 0),
          (double) (wRo(0, 0) * c + wRo(1, 0) * s)));*/

  if (atan2((double) (wRo(0, 2) * s - wRo(1, 2) * c),
          (double) (-wRo(0, 1) * s + wRo(1, 1) * c)) >= 0.) {
    roll = fmod((atan2((double) (wRo(0, 2) * s - wRo(1, 2) * c),
          (double) (-wRo(0, 1) * s + wRo(1, 1) * c))) + PI, TWOPI) - PI;
  } else {
    roll = fmod((atan2((double) (wRo(0, 2) * s - wRo(1, 2) * c),
          (double) (-wRo(0, 1) * s + wRo(1, 1) * c))) - PI, -TWOPI) + PI;
  }
  /*roll = standardRad(
      atan2((double) (wRo(0, 2) * s - wRo(1, 2) * c),
          (double) (-wRo(0, 1) * s + wRo(1, 1) * c)));*/
}

/*double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t + PI, TWOPI) - PI;
  } else {
    t = fmod(t - PI, -TWOPI) + PI;
  }
  return t;
}*/

AprilTagDetector::~AprilTagDetector(){
  image_sub_.shutdown();
  image_sub_1.shutdown();
  image_sub_2.shutdown();
}

void AprilTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  std::vector<AprilTags::TagDetection>  detections = tag_detector_->extractTags(gray);
  ROS_DEBUG("%d tag detected", (int)detections.size());

  double fx;
  double fy;
  double px;
  double py;
  if (projected_optics_) {
    // use projected focal length and principal point
    // these are the correct values
    fx = cam_info->P[0];
    fy = cam_info->P[5];
    px = cam_info->P[2];
    py = cam_info->P[6];
  } else {
    // use camera intrinsic focal length and principal point
    // for backwards compatability
    fx = cam_info->K[0];
    fy = cam_info->K[4];
    px = cam_info->K[2];
    py = cam_info->K[5];
  }

  if(!sensor_frame_id_.empty())
    cv_ptr->header.frame_id = sensor_frame_id_;

  AprilTagDetectionArray tag_detection_array;
  geometry_msgs::PoseStamped tag_pose;
  geometry_msgs::PoseArray tag_pose_array;
  tag_pose_array.header = cv_ptr->header;
  geometry_msgs::Pose2D localization_data_1;

  BOOST_FOREACH(AprilTags::TagDetection detection, detections){
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end()){
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }

    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

    std::pair<float, float> temp;
    temp.first = 0;
    temp.second = 0;
	  std::pair<float, float> * centerxy = &temp;

    detection.draw(cv_ptr->image);
    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

    //geometry_msgs::PoseStamped tag_pose;
    //tag_pose.pose.position.x = transform(0, 3);
    //tag_pose.pose.position.y = transform(1, 3);

    //Obtaining rotation matrix and translation vector
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(tag_size, fx, fy, px, py, translation, rotation);

    //turning rotation into negative inverse
    /*Eigen::Matrix3d rotation_in = rotation.inverse();
    Eigen::Matrix3d F;
    F <<
    -1, 0, 0,
    0, -1, 0,
    0, 0, -1;

    Eigen::Matrix3d rotation_neg = F * rotation_in;
    
    double yaw, pitch, roll;
    wRo_to_euler(rotation, yaw, pitch, roll);

    double new_z = transform(2, 3) * cos(roll);

    //rotate counter clockwise
    float corrected_x = (transform(0, 3) * cos(rot_quaternion.z())) - (new_z * sin(rot_quaternion.z()));
    float corrected_z = (transform(0, 3) * sin(rot_quaternion.z())) + (new_z * cos(rot_quaternion.z()));*/
    //end of counter clockwise code

    tag_pose.pose.position.x = transform(0, 3); //corrected_x; //transform(0, 3);
    tag_pose.pose.position.y = transform(1, 3);
    tag_pose.pose.position.z = transform(2, 3); //corrected_z; //new_z; //transform(2, 3)
    tag_pose.pose.orientation.x = rot_quaternion.x(); // * (180/PI);
    tag_pose.pose.orientation.y = rot_quaternion.y(); // * (180/PI);
    tag_pose.pose.orientation.z = rot_quaternion.z(); // * (180/PI);
    tag_pose.pose.orientation.w = rot_quaternion.w(); // * (180/PI);
    /*
    tag_pose.pose.orientation.x = yaw * (180/PI);
    tag_pose.pose.orientation.y = pitch * (180/PI);
    tag_pose.pose.orientation.z = roll * (180/PI);
    tag_pose.pose.orientation.w = rot_quaternion.w();*/ //* (180/PI);
    
    tag_pose.header = cv_ptr->header;

    AprilTagDetection tag_detection;
    tag_detection.pose = tag_pose;
    tag_detection.id = detection.id;
    tag_detection.size = tag_size;
    tag_detection_array.detections.push_back(tag_detection);
    tag_pose_array.poses.push_back(tag_pose.pose);

    
    /* End of Localization Code */ 

    /* Testing Code */
    float angle_approach = rawAngle(tag_pose.pose.position.x, tag_pose.pose.position.y, tag_pose.pose.position.z,
     tag_pose.pose.orientation.x, tag_pose.pose.orientation.y, tag_pose.pose.orientation.z, tag_pose.pose.orientation.w);
    

    geometry_msgs::Pose2D localization_data;
    localization_data.x = -1 * tag_pose.pose.position.x;
    localization_data.y = tag_pose.pose.position.z;
    localization_data.theta = (-1 * ((-1 * angle_approach) + (PI / 2)));
    localization_pub_.publish(localization_data);

    //geometry_msgs::Pose2D localization_data_1;
    //localization_data_1.x = (float)tag_pose.pose.position.x;
    //localization_data_1.y = (float)detection.id;
    //localization_data_1.theta = (float)tag_size;
    //localization_1_pub_.publish(localization_data_1);

    tf::Stamped<tf::Transform> tag_transform;
    tf::poseStampedMsgToTF(tag_pose, tag_transform);
    tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, description.frame_name()));
  }
  /*if ((double)tag_pose.pose.position.x == 0.0) {
    detections_pub_.publish(tag_detection_array);
    pose_pub_.publish(tag_pose_array);
    image_pub_.publish(cv_ptr->toImageMsg());
    //localization_1_pub_.publish(localization_data_1);
  }*/
  
}

void AprilTagDetector::imageCb_1(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  std::vector<AprilTags::TagDetection>  detections = tag_detector_->extractTags(gray);
  ROS_DEBUG("%d tag detected", (int)detections.size());

  double fx;
  double fy;
  double px;
  double py;
  if (projected_optics_) {
    // use projected focal length and principal point
    // these are the correct values
    fx = cam_info->P[0];
    fy = cam_info->P[5];
    px = cam_info->P[2];
    py = cam_info->P[6];
  } else {
    // use camera intrinsic focal length and principal point
    // for backwards compatability
    fx = cam_info->K[0];
    fy = cam_info->K[4];
    px = cam_info->K[2];
    py = cam_info->K[5];
  }

  if(!sensor_frame_id_.empty())
    cv_ptr->header.frame_id = sensor_frame_id_;

  AprilTagDetectionArray tag_detection_array;
  geometry_msgs::PoseArray tag_pose_array;
  geometry_msgs::PoseStamped tag_pose;
  tag_pose_array.header = cv_ptr->header;
  hci::motorCommand lookie_msg_left;

  BOOST_FOREACH(AprilTags::TagDetection detection, detections){
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end()){
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }

    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

    std::pair<float, float> temp;
    temp.first = 0;
    temp.second = 0;
    std::pair<float, float> * centerxy = &temp;

    detection.draw(cv_ptr->image);
    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

    //geometry_msgs::PoseStamped tag_pose;
    tag_pose.pose.position.x = transform(0, 3);
    tag_pose.pose.position.y = transform(1, 3);

    //Obtaining rotation matrix and translation vector
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(tag_size, fx, fy, px, py, translation, rotation);

    //turning rotation into negative inverse
    /*Eigen::Matrix3d rotation_in = rotation.inverse();
    Eigen::Matrix3d F;
    F <<
    -1, 0, 0,
    0, -1, 0,
    0, 0, -1;

    Eigen::Matrix3d rotation_neg = F * rotation_in;
    
    double yaw, pitch, roll;
    wRo_to_euler(rotation, yaw, pitch, roll);

    double new_z = transform(2, 3) * cos(roll);

    //rotate counter clockwise
    float corrected_x = (transform(0, 3) * cos(rot_quaternion.z())) - (new_z * sin(rot_quaternion.z()));
    float corrected_z = (transform(0, 3) * sin(rot_quaternion.z())) + (new_z * cos(rot_quaternion.z()));*/
    //end of counter clockwise code

    tag_pose.pose.position.x = transform(0, 3); //corrected_x; //transform(0, 3);
    tag_pose.pose.position.y = transform(1, 3);
    tag_pose.pose.position.z = transform(2, 3); //corrected_z; //new_z; //transform(2, 3)
    tag_pose.pose.orientation.x = rot_quaternion.x(); // * (180/PI);
    tag_pose.pose.orientation.y = rot_quaternion.y(); // * (180/PI);
    tag_pose.pose.orientation.z = rot_quaternion.z(); // * (180/PI);
    tag_pose.pose.orientation.w = rot_quaternion.w(); // * (180/PI);

    float angle_approach_1 = rawAngle(tag_pose.pose.position.x, tag_pose.pose.position.y, tag_pose.pose.position.z,
     tag_pose.pose.orientation.x, tag_pose.pose.orientation.y, tag_pose.pose.orientation.z, tag_pose.pose.orientation.w);
    

    geometry_msgs::Pose2D localization_data_1;
    localization_data_1.x = -1 * tag_pose.pose.position.x;
    localization_data_1.y = tag_pose.pose.position.y;
    localization_data_1.theta = (float)lookie_angle_left;//(-1 * ((-1 * angle_approach_1) + (PI / 2)));
    localization_1_pub_.publish(localization_data_1);

    tf::Stamped<tf::Transform> tag_transform;
    tf::poseStampedMsgToTF(tag_pose, tag_transform);
    tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, description.frame_name()));
  }
  /*lookie_msg_left.motorID = 6;
  lookie_msg_left.value = 0.0f;
  motor_commands_.publish(lookie_msg_left);*/
  /*if (lookie_start_pos_l) {
    lookie_start_pos_l = false;
    lookie_msg_left.motorID = 6;
    lookie_msg_left.value = 0.0f;
    motor_commands_.publish(lookie_msg_left);
  }*/
  if ((double)tag_pose.pose.position.x == 0.0) {
    if (lookie_motor_left >= 130.0f) {
      lookie_swipe_left = false;
    }
    if (lookie_motor_left <= -130.0f) {
      lookie_swipe_left = true;
    }
    if (lookie_swipe_left) {
      lookie_motor_left += 5.0f;
    }
    if (!lookie_swipe_left) {
      lookie_motor_left -= 5.0f;
    }
    lookie_msg_left.motorID = 6;
    lookie_msg_left.value = lookie_motor_left;
    //motor_commands_.publish(lookie_msg_left);
  }
}

void AprilTagDetector::imageCb_2(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  std::vector<AprilTags::TagDetection>  detections = tag_detector_->extractTags(gray);
  ROS_DEBUG("%d tag detected", (int)detections.size());

  double fx;
  double fy;
  double px;
  double py;
  if (projected_optics_) {
    // use projected focal length and principal point
    // these are the correct values
    fx = cam_info->P[0];
    fy = cam_info->P[5];
    px = cam_info->P[2];
    py = cam_info->P[6];
  } else {
    // use camera intrinsic focal length and principal point
    // for backwards compatability
    fx = cam_info->K[0];
    fy = cam_info->K[4];
    px = cam_info->K[2];
    py = cam_info->K[5];
  }

  if(!sensor_frame_id_.empty())
    cv_ptr->header.frame_id = sensor_frame_id_;

  AprilTagDetectionArray tag_detection_array;
  geometry_msgs::PoseArray tag_pose_array;
  geometry_msgs::PoseStamped tag_pose;
  tag_pose_array.header = cv_ptr->header;
  hci::motorCommand lookie_msg_right;

  BOOST_FOREACH(AprilTags::TagDetection detection, detections){
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end()){
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }

    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

    std::pair<float, float> temp;
    temp.first = 0;
    temp.second = 0;
    std::pair<float, float> * centerxy = &temp;

    detection.draw(cv_ptr->image);
    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

    geometry_msgs::PoseStamped tag_pose;
    //tag_pose.pose.position.x = transform(0, 3);
    //tag_pose.pose.position.y = transform(1, 3);

    //Obtaining rotation matrix and translation vector
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(tag_size, fx, fy, px, py, translation, rotation);

    //turning rotation into negative inverse
    /*Eigen::Matrix3d rotation_in = rotation.inverse();
    Eigen::Matrix3d F;
    F <<
    -1, 0, 0,
    0, -1, 0,
    0, 0, -1;

    Eigen::Matrix3d rotation_neg = F * rotation_in;
    
    double yaw, pitch, roll;
    wRo_to_euler(rotation, yaw, pitch, roll);

    double new_z = transform(2, 3) * cos(roll);

    //rotate counter clockwise
    float corrected_x = (transform(0, 3) * cos(rot_quaternion.z())) - (new_z * sin(rot_quaternion.z()));
    float corrected_z = (transform(0, 3) * sin(rot_quaternion.z())) + (new_z * cos(rot_quaternion.z()));*/
    //end of counter clockwise code

    tag_pose.pose.position.x = transform(0, 3); //corrected_x; //transform(0, 3);
    tag_pose.pose.position.y = transform(1, 3);
    tag_pose.pose.position.z = transform(2, 3); //corrected_z; //new_z; //transform(2, 3)
    tag_pose.pose.orientation.x = rot_quaternion.x(); // * (180/PI);
    tag_pose.pose.orientation.y = rot_quaternion.y(); // * (180/PI);
    tag_pose.pose.orientation.z = rot_quaternion.z(); // * (180/PI);
    tag_pose.pose.orientation.w = rot_quaternion.w(); // * (180/PI);

    float angle_approach_2 = rawAngle(tag_pose.pose.position.x, tag_pose.pose.position.y, tag_pose.pose.position.z,
     tag_pose.pose.orientation.x, tag_pose.pose.orientation.y, tag_pose.pose.orientation.z, tag_pose.pose.orientation.w);
    

    geometry_msgs::Pose2D localization_data_2;
    localization_data_2.x = -1 * tag_pose.pose.position.x;
    localization_data_2.y = tag_pose.pose.position.y;
    localization_data_2.theta = (float)lookie_angle_right;//(-1 * ((-1 * angle_approach_2) + (PI / 2)));
    localization_2_pub_.publish(localization_data_2);

    tf::Stamped<tf::Transform> tag_transform;
    tf::poseStampedMsgToTF(tag_pose, tag_transform);
    tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, description.frame_name()));
  }
  /*lookie_msg_right.motorID = 7;
  lookie_msg_right.value = 0.0f;
  motor_commands_.publish(lookie_msg_right);*/
  /*if (lookie_start_pos_r) {
    lookie_start_pos_r = false;
    lookie_msg_right.motorID = 7;
    lookie_msg_right.value = 0.0f;
    motor_commands_.publish(lookie_msg_right);
  }*/
  if ((double)tag_pose.pose.position.x == 0.0) {
    if (lookie_motor_right >= 130.0f) {
      lookie_swipe_right = false;
    }
    if (lookie_motor_right <= -130.0f) {
      lookie_swipe_right = true;
    }
    if (lookie_swipe_right) {
      lookie_motor_right += 5.0f;
    }
    if (!lookie_swipe_right) {
      lookie_motor_right -= 5.0f;
    }
    lookie_msg_right.motorID = 7;
    lookie_msg_right.value = lookie_motor_right;
    //motor_commands_.publish(lookie_msg_right);
  }
  
}


std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}

float AprilTagDetector::rawAngle(float position_x, float position_y, float position_z, float orientation_x, float orientation_y, float orientation_z, float orientation_w) {
  float xsqr = orientation_x * orientation_x;

    //roll(around the y)
    float t0 = +2.0f * (orientation_w * orientation_z + orientation_x * orientation_y);
    float t1 = +1.0f - 2.0f * (orientation_z * orientation_z + xsqr);
    float Mpose_pose_orientation_y = atan2(t0, t1);

    //pitch(around the x)
    float t2 = +2.0f * (orientation_w * orientation_x - orientation_y * orientation_z);
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    float Mpose_pose_orientation_x = asin(t2);

    //yaw (around the z)
    float t3 = +2.0f * (orientation_w * orientation_y + orientation_z * orientation_x);
    float t4 = +1.0f - 2.0f * (xsqr + orientation_y * orientation_y);
    float Mpose_pose_orientation_z = atan2(t3, t4);
    if (Mpose_pose_orientation_z < 0)
    {
        Mpose_pose_orientation_z += (2 * M_PI);
    }

    float Mpose_pose_orientation_w = 1.00;
    float Mpose_detected = true;

    float Mpose_pose_position_y = sqrt(position_x * position_x + position_z * position_z)
                            * cos(Mpose_pose_orientation_y - M_PI - atan(position_x / position_z));
    float Mpose_pose_position_x = sqrt(position_x * position_x + position_z * position_z)
                            * sin(Mpose_pose_orientation_y - M_PI - atan(position_x / position_z));
    float Mpose_pose_position_z = position_y;

    return Mpose_pose_orientation_z;

    //float Mpose_px = centerxy->first;
    //float Mpose_py = centerxy->second;

        /*tf::Stamped<tf::Transform> tag_transform;
    tf::Transform tag_transform_inv;
        tf::poseStampedMsgToTF(tag_pose, tag_transform);
    tag_transform_inv = tag_transform.inverse();
        if(inverse_tf)
            tf_pub_.sendTransform(tf::StampedTransform(tag_transform_inv, ros::Time::now(), description.frame_name(), tag_transform.frame_id_));
        else
      tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, description.frame_name()));*/
    /**/
}



}
