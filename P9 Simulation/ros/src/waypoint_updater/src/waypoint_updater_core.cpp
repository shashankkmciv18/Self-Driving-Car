#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include <ros/ros.h>
#include <ros/timer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "boost/date_time/posix_time/posix_time.hpp"

//#include <ecl/geometry.hpp>

#include <math.h>

// https://github.com/crvs/KDTree
#include "KDTree.hpp"

#include "waypoint_updater.h"
#include "spline.h"

inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}
inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}
inline double deg2rad(double deg)
{
  return deg * M_PI / 180;
}  // convert degree to radian


namespace waypoint_updater
{

using std::cout;
using std::endl;
using std::vector;

WaypointUpdater::WaypointUpdater(const ros::NodeHandle &handle)
{
  nh_ = handle;
  ros::NodeHandle private_nh_("~");

  private_nh_.param("/waypoint_loader/velocity", speed_param_, double(20.0));
  private_nh_.param("/simulator_mode", simulator_mode_, bool(true));

  speed_limit_ = kmph2mps(speed_param_);

  pose_set_ = false;
  waypoints_set_ = false;
  velocity_set_ = false;
  traffic_light_pos_set_ = false;

  final_waypoints_pub = nh_.advertise<styx_msgs::Lane>("/final_waypoints", 1);

  std::mutex pose_lock_;
  std::mutex velocity_lock_;
  std::mutex base_wp_lock_;
  std::mutex traffic_wp_lock_;
   
  geometry_msgs::PoseStamped pose_; 
  std_msgs::Header lane_header_;
  
  brake_dist_ = BRAKE_DISTANCE;

  vector<pointVec> waypoints_2d_;
  KDTree waypoint_tree_;

  accel_    = 0.0;
  prev_velocity_ = 0.0;
  velocity_ = 0.0;
  traffic_light_pos_ = -1;
  stop_light_pos_ = -1;
  stop_light_dist_ = 0.0;
  stop_wp_dist_ = 0;
  
  lookahead_wps_ = LOOKAHEAD_WPS; 
  vehicle_state_ = VEHICLE_STOPPED_;
  
  timer_set_ = false;
  warming_up_ = true;
  vel_time_ = ros::Time::now();
  prev_vel_time_ = ros::Time::now();
}

WaypointUpdater::~WaypointUpdater() {}

void WaypointUpdater::publish_waypoints()
{

  // ROS_INFO("pose_set_: %d, waypoints_set_: %d, velocity_set_: %d, traffic_light_pos_set_: %d", pose_set_, waypoints_set_, velocity_set_, traffic_light_pos_set_);

  if (!pose_set_ || !waypoints_set_ || !velocity_set_ || !traffic_light_pos_set_) { return; }

  pose_lock_.lock();
  geometry_msgs::PoseStamped pose = pose_;
  pose_lock_.unlock();

  base_wp_lock_.lock();
  vector<styx_msgs::Waypoint> base_waypoints = base_waypoints_;
  base_wp_lock_.unlock();
  
  traffic_wp_lock_.lock();
  int traffic_light_pos = traffic_light_pos_;
  traffic_wp_lock_.unlock();
  
  velocity_lock_.lock();

  ros::Duration dur = vel_time_ - prev_vel_time_;  
  double dur_sec = dur.toSec();

  if (dur_sec > 0.00001) {
    accel_ = (velocity_ - prev_velocity_)/dur_sec;
  } 

  double velocity = velocity_;
  prev_velocity_ = velocity;
  prev_vel_time_  = vel_time_;
  velocity_lock_.unlock();

  // ROS_INFO("In %s", __FUNCTION__);

  if (warming_up_) {
    timer_.setPeriod(ros::Duration(5.0), false);
  } 

  size_t wps_len = base_waypoints.size();
  double closest_dist;
  double light_dist = 0.0;  

  int closest_idx = get_closest_waypoint_index(pose.pose.position.x, pose.pose.position.y, closest_dist);

  size_t farest_idx = closest_idx + lookahead_wps_;

  vector<styx_msgs::Waypoint> rel_waypoints, waypoints, final_waypoints;

  if (farest_idx < wps_len) {
    for (size_t i=closest_idx; i<farest_idx; i++) {
      waypoints.push_back(base_waypoints[i]);
    }
  } else {
    for (size_t i=closest_idx; i<wps_len; i++) {
      waypoints.push_back(base_waypoints[i]);
    } 
    for (size_t i=0; i< (farest_idx%wps_len)+1; i++) {
      waypoints.push_back(base_waypoints[i]);
    }
  }

  // ROS_INFO("");

  styx_msgs::Lane lane;
  bool done = false;

  while (!done) {
    double required_stop_dist; 

    ros::Time timeNow = ros::Time::now();
    lane.header = lane_header_;
    lane.header.stamp = timeNow;

    stop_light_pos_ = traffic_light_pos_;

    double wp_pitch = 0.4;
    double CYCLE_TIME = 0.05; 
    double tSqrt = CYCLE_TIME*CYCLE_TIME;
    double prev_theta = 0.0;
    double new_theta = 0.0;

//    double plan_acc = accel_;
    double plan_acc = 0.0;
    double plan_v = velocity;
    int    plan_idx = closest_idx;
    double prev_plan_acc = plan_acc;
    double prev_plan_v = plan_v;
    double x_p = 0.0;
    double y_p = 0.0;
    double prev_x_p = 0.0;
    double prev_y_p = 0.0;
    double dist_corr = 0.0;
    double travel_dist = 0.0;
    double plan_dist = wp_pitch;
    double real_dist = wp_pitch;

    // size_t skip_wp = size_t(velocity);
    size_t skip_wp = 2;

    plan_dist = point_dist(pose.pose.position, waypoints[0].pose.pose.position);
    required_stop_dist = fabs(pow(velocity, 2)/(2.0*MAX_DECEL));

    bool stopping = false;
    bool stopped = false;

    if (stop_light_pos_==-1) {
      stop_light_dist_ = pow(10, 10);
      stopping = false;
    } else {
      stop_light_dist_  = distance(base_waypoints, closest_idx, stop_light_pos_);
      if (stop_light_dist_ <= fmax(BRAKE_DISTANCE, required_stop_dist)) {
        stopping = true;
        if (stop_light_dist_ > 0.00001 
            && get_wp_distance(base_waypoints, stop_light_pos_, closest_idx) >= 3 
            && !is_behind(wps_len, stop_light_pos_, closest_idx)) {
          plan_acc = -(pow(velocity, 2)/(2.0*stop_light_dist_));
        } else {
          stopped = true;
          plan_acc = 0.0;
        }
      } 
    }

    final_waypoints.clear();
    
    size_t i;
    for (i=0; i < lookahead_wps_-1; i++) {
      plan_idx = get_closest_waypoint_index(waypoints[i].pose.pose.position.x, waypoints[i].pose.pose.position.y, closest_dist);

      double plan_stop_dist_ = pow(10, 10);

      required_stop_dist = fabs(pow(prev_plan_v, 2)/(2.0*MAX_DECEL));

      if (stop_light_pos_ != traffic_light_pos_) {
        break;
      }

      if (!stopping) {
        double diff = prev_plan_v - speed_limit_;
        if (diff>0) {
          plan_acc = -MAX_DECEL * (1.0 - 1.0/exp(0.1*fabs(diff))) * 0.1;
        } else {
          plan_acc =  MAX_ACCEL * (1.0 - fabs(diff)/speed_limit_);
        }
      }

      double plan_vpow = pow(prev_plan_v, 2)+(2.0*plan_acc*plan_dist);
      if (plan_vpow>=0.0) {
        plan_v = sqrt(plan_vpow);
      } else {
        plan_v = 0.0;
      }

      if (stopping) {
        plan_stop_dist_ = distance(base_waypoints, plan_idx, stop_light_pos_);
        if (plan_stop_dist_ < 1.5 
            || get_wp_distance(base_waypoints, stop_light_pos_, closest_idx) < 3 
            || is_behind(wps_len, stop_light_pos_, plan_idx) 
            || stopped) { 
          plan_v = 0.0;
          plan_acc = 0.0;
          stopped = true;
        } else {
          plan_acc = -(pow(prev_plan_v, 2)/(2.0*plan_stop_dist_));
        }
        if (!stopped && plan_v < 1.0 && simulator_mode_) {
          plan_v = 1.0;
        }
      }

       // ROS_INFO("stopping: %d, stop_light_pos_: %d(%d), plan_idx: %lu, stop_light_dist: %lf,  plan_stop_dist_: %lf, velocity: %lf, plan_v: %lf, plan_acc: %lf, accel_: %lf, plan_dist: %lf, plan_vpow: %lf, is_behind: %d", stopping, stop_light_pos_, wps_len, plan_idx, stop_light_dist_, plan_stop_dist_, velocity, plan_v, plan_acc, accel_, plan_dist, plan_vpow, is_behind(wps_len, stop_light_pos_, plan_idx));

      if (plan_v > speed_limit_) { plan_v = speed_limit_; }
      if (plan_v < 0.0) { plan_v = 0.0; }

      waypoints[i].twist.twist.linear.x = plan_v;
      if (i > skip_wp) {
        final_waypoints.push_back(waypoints[i]);
      }

      prev_plan_v = plan_v;

      prev_plan_acc = plan_acc;

      plan_dist = point_dist(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position);
    }
    // ROS_INFO("");
    if (i >= lookahead_wps_-1) { done = true; }
  } 
  lane.waypoints = final_waypoints;
  final_waypoints_pub.publish(lane);
}

void WaypointUpdater::set_timer(const ros::Timer& tm)
{ 
  ros::Timer timer_ = tm;
}

// get index distance from wp1 to wp2
int WaypointUpdater::get_wp_ahead_distance(const vector<styx_msgs::Waypoint> &waypoints, size_t wp1_idx, size_t wp2_idx)
{
//  ROS_INFO("In %s", __FUNCTION__);
  size_t wps_len = waypoints.size();
  
  if (wp1_idx == wp2_idx) { return 0; }
  
  if (wp1_idx > wp2_idx) {
      return abs(wp2_idx + wps_len - wp1_idx);
  } else { 
      return abs(wp2_idx - wp1_idx);
  }
}

int WaypointUpdater::get_wp_distance(const vector<styx_msgs::Waypoint> &waypoints, size_t wp1_idx, size_t wp2_idx)
{
//  ROS_INFO("In %s", __FUNCTION__);
  size_t wps_len = waypoints.size();
  
  if (wp1_idx == wp2_idx) { return 0; }
  
  if (wp1_idx > wp2_idx) {
      return fmin(wp1_idx-wp2_idx, get_wp_ahead_distance(waypoints, wp1_idx, wp2_idx));
  } else { 
      return fmin(wp2_idx-wp1_idx, get_wp_ahead_distance(waypoints, wp2_idx, wp1_idx));
  }
}

bool WaypointUpdater::is_behind(size_t wps_len, int object_wp_idx, int vehicle_wp_idx) {
  if (object_wp_idx == vehicle_wp_idx) { return false; }
  
  if (object_wp_idx > vehicle_wp_idx) {
    if (vehicle_wp_idx+wps_len-object_wp_idx < object_wp_idx-vehicle_wp_idx) {
      return true;
    } else {
      return false;
    }
  } else {
    if (object_wp_idx+wps_len-vehicle_wp_idx < vehicle_wp_idx-object_wp_idx) {
      return false;
    } else {
      return true;
    }
  }
}

int WaypointUpdater::get_closest_waypoint_index(double x, double y, double &closest_dist)
{
//  ROS_INFO("In %s", __FUNCTION__);

  size_t wps_len = waypoints_2d_.size();

  point_t pt = {x, y};

  size_t closest_idx = waypoint_tree_.nearest_index(pt);
  
  point_t cl_vect = waypoints_2d_[closest_idx];
  point_t prev_vect = waypoints_2d_[(wps_len+closest_idx-1)%wps_len];
  
  point_t v1 = {cl_vect[0] - prev_vect[0], cl_vect[1] - prev_vect[1]};
  point_t v2 = {pt[0] - cl_vect[0], pt[1] - cl_vect[1]};
  
  double val = v1[0]*v2[0]+v1[1]*v2[1];

  if (val >= 0.0) { closest_idx = (closest_idx + 1) % wps_len; }
  
  closest_dist  = sqrt(pow(x-waypoints_2d_[closest_idx][0], 2)+pow(y-waypoints_2d_[closest_idx][1], 2));
  
  return closest_idx;
}

void WaypointUpdater::pose_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
//  ROS_INFO("In %s", __FUNCTION__);
  pose_lock_.lock();
  pose_ = *msg;
  pose_set_ = true;
  pose_lock_.unlock();
//  ROS_INFO("Exit %s", __FUNCTION__);
}

void WaypointUpdater::waypoints_cb(const styx_msgs::LaneConstPtr &msg)
{
  base_wp_lock_.lock();
  if (!waypoints_set_) {
//    ROS_INFO("In %s", __FUNCTION__);
    base_waypoints_ = msg->waypoints;
    lane_header_    = msg->header;
 
//    ROS_INFO("In %s, waypoint size: %lu", __FUNCTION__, base_waypoints_.size());
    for (size_t i=0; i<base_waypoints_.size(); i++) {
      point_t pt = {base_waypoints_[i].pose.pose.position.x, base_waypoints_[i].pose.pose.position.y};
      waypoints_2d_.push_back(pt);
    }
    waypoint_tree_ = KDTree(waypoints_2d_);
    waypoints_set_ = true;
//    ROS_INFO("Exit %s", __FUNCTION__);
  }
  base_wp_lock_.unlock();
}
 
void WaypointUpdater::velocity_cb(const geometry_msgs::TwistStampedConstPtr &msg)
{
//  ROS_INFO("In %s", __FUNCTION__);
  velocity_lock_.lock();
  velocity_ = msg->twist.linear.x;
  vel_time_ = msg->header.stamp;
  velocity_set_ = true;
  velocity_lock_.unlock();
//  ROS_INFO("Exit %s", __FUNCTION__);
}
                
void WaypointUpdater::traffic_cb(const std_msgs::Int32ConstPtr &msg)
{
//  ROS_INFO("In %s", __FUNCTION__);
  traffic_wp_lock_.lock();
  traffic_light_pos_ = msg->data;
  traffic_light_pos_set_ = true;
  traffic_wp_lock_.unlock();
//  ROS_INFO("Exit %s", __FUNCTION__);
}

void WaypointUpdater::obstacle_cb(const geometry_msgs::PoseStampedPtr &msg)
{
//  ROS_INFO("In %s", __FUNCTION__);
  // TODO: Callback for /obstacle_waypoint message. We will implement it later
}

double WaypointUpdater::point_dist(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
//  ROS_INFO("In %s", __FUNCTION__);
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

double WaypointUpdater::distance_ahead(const vector<styx_msgs::Waypoint> &waypoints, size_t start, size_t end)
{
//  ROS_INFO("In %s", __FUNCTION__);
  double dist = 0.0;
  size_t wps_len = waypoints.size();
  if (end%wps_len < start%wps_len) {
      end = end%wps_len + wps_len;
  }
  for (size_t i = start; i<end-1; i++) {
      dist += point_dist(waypoints[i%wps_len].pose.pose.position, waypoints[(i+1)%wps_len].pose.pose.position);
  }
  return dist;
}

double WaypointUpdater::distance(const vector<styx_msgs::Waypoint> &waypoints, size_t start, size_t end)
{
//  ROS_INFO("In %s", __FUNCTION__);
  double dist = 0.0;
  size_t wps_len = waypoints.size();
  double s, e;
  if (end > start) {
    s = start;
    e = end;
  } else {
    s = end;
    e = start;
  }
  for (size_t i = s; i<e-1; i++) {
      dist += point_dist(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position);
  }
  return fmin(dist, distance_ahead(waypoints, start, end));
}

geometry_msgs::Pose WaypointUpdater::calcRelativePose(const geometry_msgs::Pose &pose_msg, const geometry_msgs::Pose &ref_pose)
{ 
  tf::Transform inverse;
  tf::poseMsgToTF(ref_pose, inverse);
  tf::Transform transform = inverse.inverse();
  
  tf::Pose p;
  tf::poseMsgToTF(pose_msg, p);
  tf::Pose tf_p = transform * p;
  geometry_msgs::Pose tf_pose_msg;
  tf::poseTFToMsg(tf_p, tf_pose_msg);
  
  return tf_pose_msg;
}

geometry_msgs::Pose WaypointUpdater::calcAbsolutePose(const geometry_msgs::Pose &pose_msg, const geometry_msgs::Pose &ref_pose)
{ 
  tf::Transform inverse;
  tf::poseMsgToTF(ref_pose, inverse);
  
  tf::Pose p;
  tf::poseMsgToTF(pose_msg, p);
  tf::Pose tf_p = inverse * p;
  geometry_msgs::Pose tf_pose_msg;
  tf::poseTFToMsg(tf_p, tf_pose_msg);

  return tf_pose_msg;
}

void WaypointUpdater::timer_cb(const ros::TimerEvent &ev)
{
  ros::TimerEvent event = ev;
  
  // std::cout << "light_timer: last_expected: " << event.last_expected.toSec() << ", last_real: " << event.last_real.toSec() << ", current_expected: " << event.current_expected.toSec() << ", current_real: " << event.current_real.toSec() << std::endl; 
  timer_lock_.lock();
  warming_up_ = false;
  timer_set_ = false;
  timer_lock_.unlock();
}


}
