#ifndef BRIDGE_H
#define BRIDGE_H

// C++ includes
#include <iostream>
#include <thread>
#include <mutex>
#include <string>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/ThrottleReport.h>
#include <dbw_mkz_msgs/BrakeReport.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <cv_bridge/cv_bridge.h>

//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>


// STYX Custom messages
#include "styx_msgs/Lane.h"
#include "styx_msgs/TrafficLightArray.h"
#include "styx_msgs/TrafficLight.h"
#include "styx_msgs/Waypoint.h"

#include <uWS/uWS.h>
#include "json.hpp"

using json = nlohmann::json;

class OutMessage {
  public:
    ros::Time   stamp;
    std::string topic;
    json        data;
};

typedef void (*ServerFunc)(const std::string&, const json&); 

namespace bridge
{
class Bridge
{
  private:
    std::mutex pose_lock, veloc_lock, waypoint_lock;
    ros::NodeHandle nh;
    ros::Time prev_time;

    ServerFunc server_sendfunc;
    tf::TransformBroadcaster transBr;

    bool   prev_dbw_enable = false;
    double prev_vel = 0.0;
    bool   yaw_set = false;
    double prev_yaw = 0.0;
    double prev_angular = 0.0;

    std::mutex pvtime_lock;
    std::mutex vel_lock;
    std::mutex yaw_lock;
    std::mutex avel_lock;

    // ros::Subscriber steering_sub;
    // ros::Subscriber thrott_sub;
    // ros::Subscriber brake_sub;
    // ros::Subscriber path_sub;

    ros::Publisher  current_pose_pub;
    ros::Publisher  current_velocity_pub;
    ros::Publisher  steering_report_pub;
    ros::Publisher  throttle_report_pub;
    ros::Publisher  brake_report_pub;
    ros::Publisher  obstacle_pub;
    ros::Publisher  obstacle_points_pub;
    ros::Publisher  lidar_pub;
    ros::Publisher  trafficlights_pub;
    ros::Publisher  dbw_status_pub;
    ros::Publisher  image_pub;

  public:
    Bridge(const ros::NodeHandle &handle, const tf::TransformBroadcaster &br, const ServerFunc &svrfunc);
    ~Bridge();
  

    // for ROS
    void publish_controls(const json &data);
    void publish_obstacles(const json &data);
    void publish_lidar(const json &data);
    void publish_traffic(const json &data);
    void publish_camera(const json &data);

    void publish_odometry(const json &data);

    void callback_steering(const dbw_mkz_msgs::SteeringCmd::ConstPtr &msg);
    void callback_throttle(const dbw_mkz_msgs::ThrottleCmd::ConstPtr &msg);
    void callback_brake(const dbw_mkz_msgs::BrakeCmd::ConstPtr &msg);
    void callback_path(const styx_msgs::Lane::ConstPtr &msg);

    std::string setPrecStr(double number, int digits);
    double setPrecFloat(double number, int digits);
    styx_msgs::TrafficLight create_light(double x, double y, double z, double yaw, int state);
    geometry_msgs::PoseStamped create_pose(double x, double y, double z, double yaw);
    std_msgs::Float32 create_float(double val);
    geometry_msgs::TwistStamped create_twist(double velocity, double angular, const ros::Time &tm);
    dbw_mkz_msgs::SteeringReport create_steer(double val);
    double calc_angular(ros::Time time, double yw);
    sensor_msgs::PointCloud2 create_point_cloud_message(const json &pts);
    void broadcast_transform(const std::string &name, double x, double y, double z, double yaw);
    void publish_dbw_status(const json &data);

};


}

#endif  // BRIDGE_H
