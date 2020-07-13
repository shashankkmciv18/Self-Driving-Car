#include <iostream>
#include <string>
#include <thread>
#include <mutex>
// #include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_ros/transform_broadcaster.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/pcl_nodelet.h>

#include <uWS/uWS.h>
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#include "bridge.h"

// Base64 utilities from https://blog.csdn.net/how0723/article/details/80831280

static std::string base64Decode(const char* Data, int DataByte)
{
	const char DecodeTable[] =
	{
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		62, // '+'
		0, 0, 0,
		63, // '/'
		52, 53, 54, 55, 56, 57, 58, 59, 60, 61, // '0'-'9'
		0, 0, 0, 0, 0, 0, 0,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
		13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, // 'A'-'Z'
		0, 0, 0, 0, 0, 0,
		26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
		39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, // 'a'-'z'
	};
	std::string strDecode;
	int nValue;
	int i = 0;
	while (i < DataByte)
	{
		if (*Data != '\r' && *Data != '\n')
		{
			nValue = DecodeTable[*Data++] << 18;
			nValue += DecodeTable[*Data++] << 12;
			strDecode += (nValue & 0x00FF0000) >> 16;
			if (*Data != '=')
			{
				nValue += DecodeTable[*Data++] << 6;
				strDecode += (nValue & 0x0000FF00) >> 8;
				if (*Data != '=')
				{
					nValue += DecodeTable[*Data++];
					strDecode += nValue & 0x000000FF;
				}
			}
			i += 4;
		}
		else
		{
			Data++;
			i++;
		}
	}
	return strDecode;
}
 
 
static std::string base64Encode(const unsigned char* Data, int DataByte)
{
	const char EncodeTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	std::string strEncode;
	unsigned char Tmp[4] = { 0 };
	int LineLength = 0;
	for (int i = 0; i < (int)(DataByte / 3); i++)
	{
		Tmp[1] = *Data++;
		Tmp[2] = *Data++;
		Tmp[3] = *Data++;
		strEncode += EncodeTable[Tmp[1] >> 2];
		strEncode += EncodeTable[((Tmp[1] << 4) | (Tmp[2] >> 4)) & 0x3F];
		strEncode += EncodeTable[((Tmp[2] << 2) | (Tmp[3] >> 6)) & 0x3F];
		strEncode += EncodeTable[Tmp[3] & 0x3F];
		if (LineLength += 4, LineLength == 76) { strEncode += "\r\n"; LineLength = 0; }
	}
	int Mod = DataByte % 3;
	if (Mod == 1)
	{
		Tmp[1] = *Data++;
		strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
		strEncode += EncodeTable[((Tmp[1] & 0x03) << 4)];
		strEncode += "==";
	}
	else if (Mod == 2)
	{
		Tmp[1] = *Data++;
		Tmp[2] = *Data++;
		strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
		strEncode += EncodeTable[((Tmp[1] & 0x03) << 4) | ((Tmp[2] & 0xF0) >> 4)];
		strEncode += EncodeTable[((Tmp[2] & 0x0F) << 2)];
		strEncode += "=";
	}
 
	return strEncode;
}
 
 
std::string Mat2Base64(const cv::Mat &img, std::string imgType)
{
	std::string img_data;
	std::vector<uchar> vecImg;
	std::vector<int> vecCompression_params;
	vecCompression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	vecCompression_params.push_back(90);
	imgType = "." + imgType;
	cv::imencode(imgType, img, vecImg, vecCompression_params);
	img_data = base64Encode(vecImg.data(), vecImg.size());
	return img_data;
}
 
 
cv::Mat Base2Mat(std::string &base64_data)
{
	cv::Mat img;
	std::string s_mat;
	s_mat = base64Decode(base64_data.data(), base64_data.size());
	std::vector<char> base64_img(s_mat.begin(), s_mat.end());
	img = cv::imdecode(base64_img, CV_LOAD_IMAGE_COLOR);
	return img;
}

// End of Base64 utilities from https://blog.csdn.net/how0723/article/details/80831280

namespace bridge
{

Bridge::Bridge(const ros::NodeHandle &handle, const tf::TransformBroadcaster &br, const ServerFunc &svrfunc)
{
  nh = handle;
  server_sendfunc = svrfunc;
  prev_vel = 0.0;
  yaw_set = false;
  prev_yaw = 0.0;
  prev_angular = 0.0;
  prev_time = ros::Time::now();
  transBr = br;

  current_pose_pub      = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1);
  current_velocity_pub  = nh.advertise<geometry_msgs::TwistStamped>("/current_velocity", 1);
  steering_report_pub   = nh.advertise<dbw_mkz_msgs::SteeringReport>("/vehicle/steering_report", 1);
  throttle_report_pub   = nh.advertise<dbw_mkz_msgs::ThrottleReport>("/vehicle/throttle_report", 1); 
  brake_report_pub      = nh.advertise<dbw_mkz_msgs::BrakeReport>("/vehicle/brake_report", 1);
  obstacle_pub          = nh.advertise<geometry_msgs::PoseStamped>("/vehicle/obstacle", 1); 
  obstacle_points_pub   = nh.advertise<sensor_msgs::PointCloud2>("/vehicle/obstacle_points", 1); 
  lidar_pub             = nh.advertise<sensor_msgs::PointCloud2>("/vehicle/lidar", 1); 
  trafficlights_pub     = nh.advertise<styx_msgs::TrafficLightArray>("/vehicle/traffic_lights", 1); 
  dbw_status_pub        = nh.advertise<std_msgs::Bool>("/vehicle/dbw_enabled", 1);
  image_pub             = nh.advertise<sensor_msgs::Image>("/image_color", 1);
}

Bridge::~Bridge() {}

std::string Bridge::setPrecStr(double number, int digits)
{
  double base = pow(10.0, digits);
  return std::to_string(double(int(number*base))/base);
}

double Bridge::setPrecFloat(double number, int digits)
{
  double base = pow(10.0, digits);
  return double(int(number*base))/base;
}

styx_msgs::TrafficLight Bridge::create_light(double x, double y, double z, double yaw, int state)
{
  styx_msgs::TrafficLight light;
  
  light.header.stamp = ros::Time::now();
  light.header.frame_id = "/world";
  
  light.pose = create_pose(x, y, z, yaw);
  light.state = state;

  return light;
}

geometry_msgs::PoseStamped Bridge::create_pose(double x, double y, double z, double yaw)
{
  geometry_msgs::PoseStamped pose;
  tf::Quaternion q;
  
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/world";
  
  pose.pose.position.x = x; 
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  // q.setRPY(0.0, 0.0, yaw * M_PI/180.0);
  // pose.pose.orientation.x = q.x();
  // pose.pose.orientation.y = q.y();
  // pose.pose.orientation.z = q.z();
  // pose.pose.orientation.w = q.w();
  
  return pose;
}

std_msgs::Float32 Bridge::create_float(double val)
{
  std_msgs::Float32 fl;

  fl.data = val;

  return fl;
}

geometry_msgs::TwistStamped Bridge::create_twist(double velocity, double angular, const ros::Time &tm)
{
  geometry_msgs::TwistStamped tw;
  
  tw.header.stamp = tm;
  tw.twist.linear.x = velocity;
  tw.twist.angular.z = angular;
  
  return tw;
}

dbw_mkz_msgs::SteeringReport Bridge::create_steer(double val)
{
  dbw_mkz_msgs::SteeringReport st;

  st.steering_wheel_angle = val * M_PI/180.0;
  st.enabled = true;
  vel_lock.lock();
  st.speed = prev_vel;
  vel_lock.unlock();
  st.header.stamp = ros::Time::now();

  return st;
}

double Bridge::calc_angular(ros::Time time, double yaw)
{
  double angular_vel = 0.0;
  ros::Duration dur = time - prev_time;

  avel_lock.lock();
  if (yaw_set) {
      angular_vel = (yaw - prev_yaw)/dur.toSec();
  }
  avel_lock.unlock();

  return angular_vel;
}


sensor_msgs::PointCloud2 Bridge::create_point_cloud_message(const json &pts)
{
  sensor_msgs::PointCloud2 msg_p2;
  sensor_msgs::PointCloud msg;

  // TODO: 
  // Translate the following python codes to C++
  // 
  // header = Header()
  // header.stamp = rospy.Time.now()
  // header.frame_id = '/world'
  // cloud_message = pcl2.create_cloud_xyz32(header, pts)
  // return cloud_message

  // msg.header.stamp = ros::Time::now();
  // msg.header.frame_id = "/world";
  // msg.height = msg.width = pts.size();
  for (unsigned int i=0; i<pts.size(); i++) {
    double x = pts[i][0];
    double y = pts[i][1];
    double z = pts[i][2];
  //  msg.points.push_back(pcl::PointXYZ(x, y, z));
  }

  sensor_msgs::convertPointCloudToPointCloud2(msg, msg_p2);

  return msg_p2;
}

void Bridge::broadcast_transform(const std::string &name, double x, double y, double z, double yaw)
{
  // static tf::TransformBroadcaster br;

  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(x, y, z));
  q.setRPY(0, 0, yaw);
  transform.setRotation(q);
  transBr.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
}

void Bridge::publish_odometry(const json &data)
{
  geometry_msgs::PoseStamped pose;

  double x = data["x"];
  double y = data["y"];
  double z = data["z"];
  double yaw = data["yaw"];
  double velocity = data["velocity"];

  using std::string; 

  double yaw_tmp =  yaw * M_PI/180.0;

  pose = create_pose(x, y, z, yaw_tmp);
  
  broadcast_transform("base_link", x, y, z, yaw_tmp);
  
  ros::Time nowTime = ros::Time::now();
  pose.header.stamp = nowTime;
  pose.header.frame_id = "/world";
  current_pose_pub.publish(pose);
  
  double curr_vel = velocity * 0.44704;
  double angular = calc_angular(nowTime, yaw_tmp);

  current_velocity_pub.publish(create_twist(curr_vel, angular, nowTime));

  yaw_lock.lock();
  prev_yaw = yaw_tmp;
  yaw_set = true;
  yaw_lock.unlock();

  vel_lock.lock();
  prev_vel = curr_vel;
  vel_lock.unlock();

  avel_lock.lock();
  prev_angular = angular;
  avel_lock.unlock();

  pvtime_lock.lock();
  prev_time = nowTime;
  pvtime_lock.unlock();
}

void Bridge::publish_controls(const json &data)
{
  double steering = data["steering_angle"];
  double throttle = data["throttle"];
  double brake    = data["brake"];

  steering_report_pub.publish(create_steer(steering));
  throttle_report_pub.publish(create_float(throttle));
  brake_report_pub.publish(create_float(brake));
}

void Bridge::publish_obstacles(const json &data)
{
  sensor_msgs::PointCloud2 cloud_msg;

  for (unsigned int i=0; i<data["obstacles"].size(); i++) {
    double x = data["obstacles"][i][0];
    double y = data["obstacles"][i][1];
    double z = data["obstacles"][i][2];
    geometry_msgs::PoseStamped pose = create_pose(x, y, z, 0.0);
    pose.header.stamp = ros::Time::now();

    obstacle_pub.publish(pose);
  }

  cloud_msg = create_point_cloud_message(data["obstacles"]); 

  obstacle_points_pub.publish(cloud_msg);
}

void Bridge::publish_lidar(const json &data)
{
  sensor_msgs::PointCloud2 cloud_msg;

  std::vector<double> x = data["lidar_x"];
  std::vector<double> y = data["lidar_y"];
  std::vector<double> z = data["lidar_z"];

  json pts;
  for (unsigned int i=0; i<x.size(); i++) {
    pts[i][0] = x[i];
    pts[i][1] = y[i];
    pts[i][2] = z[i];
  }

  cloud_msg = create_point_cloud_message(pts); 
  
  lidar_pub.publish(cloud_msg);
}

void Bridge::publish_traffic(const json &data)
{
  std::vector<double> x_pos = data["light_pos_x"];
  std::vector<double> y_pos = data["light_pos_y"];
  std::vector<double> z_pos = data["light_pos_z"];
  std::vector<double> dx    = data["light_pos_dx"];
  std::vector<double> dy    = data["light_pos_dy"];
  std::vector<int> states = data["light_state"];
  styx_msgs::TrafficLightArray lights;

  lights.header.stamp = ros::Time::now();
  lights.header.frame_id = "/world";
  
  for (unsigned int i=0; i<x_pos.size(); i++) {
    double x = x_pos[i];
    double y = y_pos[i];
    double z = z_pos[i];
    double yaw = atan2(dy[i], dx[i]);
    int st =  states[i];
    lights.lights.push_back(create_light(x, y, z, yaw, st));
  }

  trafficlights_pub.publish(lights);
}

void Bridge::publish_dbw_status(const json &data)
{
  std_msgs::Bool msg;
  
  prev_dbw_enable = data["dbw_enable"];
  msg.data = prev_dbw_enable;
  dbw_status_pub.publish(msg);
}

void Bridge::publish_camera(const json &data)
{
  std_msgs::Header header;

  std::string imgString = data["image"];

  header.stamp = ros::Time::now();
  header.frame_id = "/world";

  cv::Mat image = Base2Mat(imgString);
  cv_bridge::CvImage cImg(header, "bgr8", image);
  
  image_pub.publish(cImg.toImageMsg());
}

void Bridge::callback_steering(const dbw_mkz_msgs::SteeringCmdConstPtr &msg)
{
  json data;
  
  // The value must be quoted.  Make it shorter to avoid stalling the simulator.
  data["steering_angle"] = setPrecStr(msg->steering_wheel_angle_cmd, 4);
  server_sendfunc("steer", data);
}

void Bridge::callback_throttle(const dbw_mkz_msgs::ThrottleCmdConstPtr &msg)
{
  json data;

  // The value must be quoted.  Make it shorter to avoid stalling the simulator.
  data["throttle"] = setPrecStr(msg->pedal_cmd, 4);
  server_sendfunc("throttle", data);
}

void Bridge::callback_brake(const dbw_mkz_msgs::BrakeCmdConstPtr &msg)
{
  json data;

  // The value must be quoted.  Make it shorter to avoid stalling the simulator.
  data["brake"] = setPrecStr(msg->pedal_cmd, 4);
  server_sendfunc("brake", data);
}

void Bridge::callback_path(const styx_msgs::LaneConstPtr &msg)
{
  json data;
  std::vector<double> x_values;
  std::vector<double> y_values;
  std::vector<double> z_values;

  for (std::size_t i=0; i < msg->waypoints.size(); i++) {
    double x = msg->waypoints[i].pose.pose.position.x;
    double y = msg->waypoints[i].pose.pose.position.y;
    double z = msg->waypoints[i].pose.pose.position.z+0.5;
    x_values.push_back(setPrecFloat(x, 4));
    y_values.push_back(setPrecFloat(y, 4));
    z_values.push_back(setPrecFloat(z, 4));
  }

  // Values are unquoted
  data["next_x"] = x_values;
  data["next_y"] = y_values;
  data["next_z"] = z_values;
  server_sendfunc("drawline", data);
}

}
