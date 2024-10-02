#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>


#include <nav_msgs/Odometry.h>

class OdomHelper{
  private:
  // tf2
  boost::shared_ptr<tf2_ros::Buffer> tfBuffer_; // tf buffer - use for lookup transform between frames
  boost::shared_ptr<tf2_ros::TransformListener> tf_listener_; // helper to subscribe to tf tree

  std::string odom_frame_;
  std::string base_frame_; 
  std::string imu_frame_;

  public:
  OdomHelper();
  ~OdomHelper();
  void init(std::string odom_frame, std::string imu_frame, std::string base_frame);
  bool getTransformStamped(std::string target_frame, std::string source_frame, geometry_msgs::TransformStamped& transformStamped);
  bool estimateEgocentric(const nav_msgs::Odometry& odom_in, nav_msgs::Odometry& odom_out);
};