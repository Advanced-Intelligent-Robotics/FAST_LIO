#include "odomHelper.h"

OdomHelper::OdomHelper(){
  ROS_INFO("Odom Helper Constructor");
}
OdomHelper::~OdomHelper(){
  ROS_INFO("Odom Helper Destructor");
}

void OdomHelper::init(std::string odom_frame, std::string lidar_frame, std::string base_frame){
  ROS_INFO("Initializing Odom Helper");
  odom_frame_ = odom_frame;
  base_frame_ = base_frame;
  lidar_frame_ = lidar_frame;
  // initialize tf_listener
  tfBuffer_ = boost::make_shared<tf2_ros::Buffer>();
  tf_listener_ = boost::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

bool OdomHelper::estimateEgocentric(const nav_msgs::Odometry& odom_in, nav_msgs::Odometry& odom_out){
  try{
    // 1. Get transformation from base_footprint -> lidar
    geometry_msgs::TransformStamped base_to_lidar;
    tf2::Transform tf_base_to_lidar;
    base_to_lidar = tfBuffer_->lookupTransform(lidar_frame_,base_frame_,ros::Time(0)); // Lookup tf for base -> lidar
    tf2::fromMsg(base_to_lidar.transform,tf_base_to_lidar); // convert to tf2::Transform

    // 2. 
    geometry_msgs::TransformStamped odom_to_lidar;
    tf2::Transform tf_odom_to_lidar;
    odom_to_lidar.header.stamp = odom_in.header.stamp;
    odom_to_lidar.header.frame_id = odom_frame_;           // Parent frame
    odom_to_lidar.child_frame_id = lidar_frame_;  // Child frame (could be base_link or another)
    odom_to_lidar.transform.translation.x = odom_in.pose.pose.position.x;
    odom_to_lidar.transform.translation.y = odom_in.pose.pose.position.y;
    odom_to_lidar.transform.translation.z = odom_in.pose.pose.position.z;
    odom_to_lidar.transform.rotation = odom_in.pose.pose.orientation;
    tf2::fromMsg(odom_to_lidar.transform,tf_odom_to_lidar);

    // calculate transformation odom -> base
    tf2::Transform tf_odom_to_base = tf_base_to_lidar * tf_odom_to_lidar * tf_base_to_lidar.inverse();
    geometry_msgs::TransformStamped odom_to_base;
    odom_to_base.header.stamp = odom_in.header.stamp;
    odom_to_base.header.frame_id = odom_frame_;
    odom_to_base.child_frame_id = base_frame_;
    odom_to_base.transform = tf2::toMsg(tf_odom_to_base);

    // Convert to odom msg
    odom_out.header.stamp = odom_to_base.header.stamp;
    odom_out.header.frame_id = odom_frame_;
    odom_out.child_frame_id = base_frame_;
    odom_out.pose.pose.position.x = odom_to_base.transform.translation.x;
    odom_out.pose.pose.position.y = odom_to_base.transform.translation.y;
    odom_out.pose.pose.position.z = odom_to_base.transform.translation.z;
    odom_out.pose.pose.orientation = odom_to_base.transform.rotation;
    odom_out.pose.covariance = odom_in.pose.covariance;
    
    return true;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("Could not compute transformation: %s", ex.what());
    return false;
  }
  
}