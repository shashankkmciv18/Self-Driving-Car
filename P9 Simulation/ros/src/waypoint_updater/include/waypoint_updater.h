#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>

#include <math.h>

// STYX Custom messages
#include "styx_msgs/Lane.h"
#include "styx_msgs/TrafficLightArray.h"
#include "styx_msgs/TrafficLight.h"
#include "styx_msgs/Waypoint.h"

// https://github.com/crvs/KDTree
#include "KDTree.hpp"

#define LOOKAHEAD_WPS  200
#define MAX_DECEL  1.0  //  Maximum Deceleration value
#define BRAKE_DISTANCE  120.0  //  distance to begin slowing down
#define MAX_ACCEL  1.0  //  Maximum Acceleration value

enum VehicleState { VEHICLE_STOPPED_, VEHICLE_MOVING_, VEHICLE_STOPPING_};

namespace waypoint_updater
{
class WaypointUpdater 
{
  private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  double speed_param_;  
  double speed_limit_;

  ros::Publisher final_waypoints_pub;

  std::mutex pose_lock_;
  std::mutex velocity_lock_;
  std::mutex base_wp_lock_;
  std::mutex traffic_wp_lock_;
  std::mutex timer_lock_;
   
  geometry_msgs::PoseStamped pose_; 
  std::vector<styx_msgs::Waypoint> base_waypoints_;
  int traffic_light_pos_;
  double prev_velocity_;
  double velocity_;
  double brake_dist_;

  pointVec waypoints_2d_;
  KDTree waypoint_tree_;

  double accel_;

  ros::Timer timer_;

  std_msgs::Header lane_header_;
  size_t lookahead_wps_;

  bool pose_set_;
  bool waypoints_set_;
  bool velocity_set_;
  bool traffic_light_pos_set_;
  VehicleState vehicle_state_;
  int  stop_light_pos_;
  double stop_light_dist_; 

  bool timer_set_;
  bool warming_up_;
  bool simulator_mode_;
  int stop_wp_dist_;

  ros::Time prev_time_;
  ros::Time vel_time_;
  ros::Time prev_vel_time_;
  
  styx_msgs::Waypoint stop_light_wp_;

  public:

  WaypointUpdater(const ros::NodeHandle &handle);
  ~WaypointUpdater();
  void publish_waypoints();
  void pose_cb(const geometry_msgs::PoseStampedConstPtr &msg);
  void waypoints_cb(const styx_msgs::LaneConstPtr &msg);
  void velocity_cb(const geometry_msgs::TwistStampedConstPtr &msg);
  void traffic_cb(const std_msgs::Int32ConstPtr &msg);
  void obstacle_cb(const geometry_msgs::PoseStampedPtr &msg);

  int get_wp_distance(const std::vector<styx_msgs::Waypoint> &waypoints, size_t wp1_idx, size_t wp2_idx);
  int get_wp_ahead_distance(const std::vector<styx_msgs::Waypoint> &waypoints, size_t wp1_idx, size_t wp2_idx);
  int get_closest_waypoint_index(double x, double y, double &closest_dist);
  double point_dist(const geometry_msgs::Point &a, const geometry_msgs::Point &b);
  double distance_ahead(const std::vector<styx_msgs::Waypoint> &waypoints, size_t start, size_t end);
  double distance(const std::vector<styx_msgs::Waypoint> &waypoints, size_t start, size_t end);
  geometry_msgs::Pose calcRelativePose(const geometry_msgs::Pose &pose_msg, const geometry_msgs::Pose &ref_pose);
  geometry_msgs::Pose calcAbsolutePose(const geometry_msgs::Pose &pose_msg, const geometry_msgs::Pose &ref_pose);
  bool is_behind(size_t wps_len, int object_wp_idx, int vehicle_wp_idx);
  void timer_cb(const ros::TimerEvent &ev);
  void set_timer(const ros::Timer& timer);
};
}
#endif
