
// ROS includes
#include <ros/ros.h>

// C++ includes
#include <iostream>
#include <thread>
#include <mutex>
#include <string>

#include "waypoint_updater.h"

using std::cout;
using std::string;
using std::vector;
using std::map;
using std::cout;
using std::endl;

int main(int argc, char *argv[]) 
{

  ros::init(argc, argv, "waypoint_updater");
  ros::NodeHandle nh;
  ros::Timer ros_timer;
  
  waypoint_updater::WaypointUpdater wpUpdater(nh);

  ros_timer = nh.createTimer(ros::Duration(20.0), &waypoint_updater::WaypointUpdater::timer_cb, &wpUpdater);
  wpUpdater.set_timer(ros_timer);

  cout << "Subscribing /current_pose" << endl;
  ros::Subscriber pose_cb       = nh.subscribe("/current_pose", 1, &waypoint_updater::WaypointUpdater::pose_cb, &wpUpdater);
  cout << "Subscribing /base_waypoints" << endl;
  ros::Subscriber waypoints_cb  = nh.subscribe("/base_waypoints", 1, &waypoint_updater::WaypointUpdater::waypoints_cb, &wpUpdater);
  cout << "Subscribing /traffic_waypoint" << endl;
  ros::Subscriber traffic_cb    = nh.subscribe("/traffic_waypoint", 1, &waypoint_updater::WaypointUpdater::traffic_cb, &wpUpdater);
  cout << "Subscribing /current_velocity" << endl;
  ros::Subscriber velocity_cb   = nh.subscribe("/current_velocity", 1, &waypoint_updater::WaypointUpdater::velocity_cb, &wpUpdater);

  ros::Rate loop_rate(50);
  ros::AsyncSpinner spinner(5);
  spinner.start();
  while (ros::ok())
  {
    ros::spinOnce();
    try {
      wpUpdater.publish_waypoints();
    } catch (const std::exception& e) {
        cout << e.what() << endl;
    }
    loop_rate.sleep();
  }
  ros::waitForShutdown();
}
