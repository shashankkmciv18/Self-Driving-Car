
// ROS includes
#include <ros/ros.h>

// C++ includes
#include <iostream>
#include <thread>
#include <mutex>
#include <string>

#include <uWS/uWS.h>
#include "json.hpp"
#include "bridge.h"

using std::cout;
using std::string;
using std::vector;
using std::map;
using std::cout;
using std::endl;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
  std::size_t lb = s.find_first_of('[');
  std::size_t rb = string::npos;
  string result = "";
  if (lb != string::npos) {
    int lcnt = 1;
    int rcnt = 0;
    std::size_t slen = s.length();

    for (std::size_t i=lb+1; i<slen; i++) {
      if (s[i] == ']') { rcnt ++; }
      if (lcnt == rcnt) { 
        rb = i;
        break; 
      };
      if (s[i] == '[') { lcnt ++; }
    }
    if (lcnt == rcnt) {
      result = s.substr(lb, rb-lb+1);
    } else {
      result = "";
    }
  }
  return result;
}

std::vector<OutMessage> outgoingMesgs;
std::vector<OutMessage> outgoingWayps;
std::mutex send_lock;

void send(const string &topic, const json &data)
{
  OutMessage m;

  m.topic = topic;
  m.data = data;
  m.stamp = ros::Time::now();

  try {
    send_lock.lock();
    if (topic == "drawline") {
        outgoingWayps.push_back(m);
    } else {
        outgoingMesgs.push_back(m);
    }
    send_lock.unlock();
  } catch (const std::exception& e) { 
     std::cout << e.what() << std::endl;
  }
}

int main(int argc, char *argv[]) {
  uWS::Hub h;
  bool dbw_enable = false;

  ros::init(argc, argv, "styx_server");
  ros::NodeHandle nh;

  static tf::TransformBroadcaster br;

  double prev_steer = -1000.0;
  double prev_throt = -1000.0;
  double prev_brake = -1000.0;
  
  bridge::Bridge bridge(nh, br, send);

  ROS_INFO("Set subscribers...");
  // subscribe topic
  ros::Subscriber steering_sub  = nh.subscribe("/vehicle/steering_cmd", 1, &bridge::Bridge::callback_steering, &bridge);
  ros::Subscriber thrott_sub    = nh.subscribe("/vehicle/throttle_cmd", 1, &bridge::Bridge::callback_throttle, &bridge);
  ros::Subscriber brake_sub     = nh.subscribe("/vehicle/brake_cmd", 1, &bridge::Bridge::callback_brake, &bridge);
  ros::Subscriber path_sub      = nh.subscribe("/final_waypoints", 1, &bridge::Bridge::callback_path, &bridge);
  ROS_INFO("Done...");

  h.onMessage([&bridge,&dbw_enable,&prev_steer,&prev_throt,&prev_brake]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
               uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data));

      if (s != "") {
        // cout << "s: " << s << endl;
        auto data = json::parse(s);

        string event = data[0].get<string>();
        
        if (event == "telemetry") {
         // std::cout << "In " << event << std::endl;
          // j[1] is the data JSON object

          if (data[1]["dbw_enable"] != dbw_enable) {
            dbw_enable = data[1]["dbw_enable"];
            bridge.publish_dbw_status(data[1]);
          }

          bridge.publish_odometry(data[1]);
          
        } else if (event == "control") {
            // std::cout << "In " << event << std::endl;
            bridge.publish_controls(data[1]);
        } else if (event == "obstacle") {
            // std::cout << "In " << event << std::endl;
            bridge.publish_obstacles(data[1]);
        } else if (event == "lidar") {
            // std::cout << "In " << event << std::endl;
            bridge.publish_lidar(data[1]);
        } else if (event == "trafficlights") {
            // std::cout << "In " << event << std::endl;
            bridge.publish_traffic(data[1]);
        } else if (event == "image") {
            // std::cout << "In " << event << std::endl;
            bridge.publish_camera(data[1]);
        }

        send_lock.lock();
        std::vector<OutMessage> outMesgs = outgoingMesgs;
        std::vector<OutMessage> outWayps = outgoingWayps;
        outgoingMesgs.clear();
        outgoingWayps.clear();
        send_lock.unlock();
        if (dbw_enable) {
          for (std::size_t i=0; i<outMesgs.size(); i++) {
            auto msg = "42[\"" + outMesgs[i].topic + "\","+ outMesgs[i].data.dump()+"]";
            if (outMesgs[i].topic == "steer") {
              double curr_steer = std::stod(outMesgs[i].data["steering_angle"].get<string>());
              if (fabs(prev_steer-curr_steer) > 0.0001) {
                prev_steer = curr_steer;
                // ROS_INFO("steer: %s", msg.c_str());
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              } 
            } else if (outMesgs[i].topic == "throttle") {
              double curr_throt = std::stod(outMesgs[i].data["throttle"].get<string>());
              if (fabs(prev_throt-curr_throt) > 0.0001) {
                prev_throt = curr_throt;
                // ROS_INFO("throttle: %s", msg.c_str());
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              } 
            } else if (outMesgs[i].topic == "brake") {
              double curr_brake = std::stod(outMesgs[i].data["brake"].get<string>());
              if (fabs(prev_brake-curr_brake) > 0.0001) {
                prev_brake = curr_brake;
                // ROS_INFO("brake: %s", msg.c_str());
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              } 
            }
          }
        }
        // Send out just the last batch of waypoints
        if (outWayps.size()>0) {
          auto wpmsg = "42[\"" + outWayps.back().topic + "\","+ outWayps.back().data.dump()+"]";
          // cout << "drawline: " << wpmsg << endl;
          ws.send(wpmsg.data(), wpmsg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if

  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    
    h.getDefaultGroup<uWS::SERVER>().close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  try {
    ros::AsyncSpinner spinner(4); 
    spinner.start();
    h.run();
  }
  catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
  }
  ros::waitForShutdown();
}
