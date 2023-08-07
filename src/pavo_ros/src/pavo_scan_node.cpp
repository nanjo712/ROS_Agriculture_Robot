#include "pavo_driver.h"
#include "std_srvs/Empty.h"
#include <dynamic_reconfigure/server.h>
#include <fstream>
#include <pavo_ros/pavo_ros_cfgConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define COMP_NODES (36000)
#define CIRCLE_ANGLE (27000.0)
#define START_ANGLE (4500)

#define DEG2RAD(x) ((x)*M_PI / 180.)

using namespace pavo;
pavo::pavo_driver *drv = NULL;

void publish_msg(ros::Publisher *pub,
                 std::vector<pavo_response_scan_t> &nodes_vec, ros::Time start,
                 double scan_time, std::string frame_id, bool inverted,
                 double angle_min, double angle_max, double min_range,
                 double max_range, bool switch_active_mode, int method) 
{
  sensor_msgs::LaserScan scanMsg;
  size_t node_count = nodes_vec.size();
  int counts = node_count * ((angle_max - angle_min) / 270.0f);
  int angle_start = 135 + angle_min;
  int node_start = node_count * (angle_start / 270.0f);

  scanMsg.ranges.resize(counts);
  scanMsg.intensities.resize(counts);

  float range = 0.0;
  float intensity = 0.0;

  for (size_t i = 0; i < counts; i++) 
  {
    range = nodes_vec[node_start].distance * 0.002;
    intensity = nodes_vec[node_start].intensity;
    if ((range > max_range) || (range < min_range)) 
    {
        range = 0.0;
        intensity = 0.0;
    }
    if (!inverted) 
    {
        scanMsg.ranges[i] = range;
        scanMsg.intensities[i] = intensity;
        node_start = node_start + 1;
    } 
    else 
    {
        scanMsg.ranges[counts - 1 - i] = range;
        scanMsg.intensities[counts - 1 - i] = intensity;
        node_start = node_start + 1;
    }
  }
  scanMsg.header.stamp = start;
  scanMsg.header.frame_id = frame_id;
  scanMsg.angle_min = Degree2Radians(angle_min);
  scanMsg.angle_max = Degree2Radians(angle_max);
  scanMsg.angle_increment = (scanMsg.angle_max - scanMsg.angle_min) / (double)counts;
  scanMsg.scan_time = scan_time;
  scanMsg.time_increment = scan_time / (double)node_count;
  scanMsg.range_min = min_range;
  scanMsg.range_max = max_range;
  pub->publish(scanMsg);
}

void callback(pavo_ros::pavo_ros_cfgConfig &config, uint32_t level) 
{
  ROS_INFO("config lidar's param:angle_min:%.2f  angle_max:%.2f  "
           "range_min:%.2f  range_max:%.2f  frame_id:%s  inverted:%s  "
           "method:%d",
           config.angle_min, 
           config.angle_max, 
           config.range_min,
           config.range_max, 
           config.frame_id.c_str(),
           config.inverted ? "True" : "False", config.method);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pavo_scan_node");
  std::string frame_id, lidar_ip, host_ip, scan_topic;
  int lidar_port, host_port;
  bool inverted, enable_motor, switch_active_mode;
  int motor_speed,motor_speed_get;
  int merge_coef;
  double angle_min;
  double angle_max;
  double max_range, min_range;
  int method;

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh("~");

  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
  nh_private.param<std::string>("scan_topic", scan_topic, "scan");
  nh_private.param<double>("angle_max", angle_max, 135.00);
  nh_private.param<double>("angle_min", angle_min, -135.00);
  nh_private.param<double>("range_max", max_range, 20.0);
  nh_private.param<double>("range_min", min_range, 0.10);
  nh_private.param<bool>("inverted", inverted, false);
  nh_private.param<int>("motor_speed", motor_speed, 15);
  nh_private.param<int>("merge_coef", merge_coef, 2);
  nh_private.param<bool>("enable_motor", enable_motor, true);
  nh_private.param<std::string>("lidar_ip", lidar_ip, "10.10.10.101");
  nh_private.param<int>("lidar_port", lidar_port, 2368);
  nh_private.param<std::string>("host_ip", host_ip, "10.10.10.100");
  nh_private.param<int>("host_port", host_port, 2368);
  nh_private.param<int>("method", method, 0);
  nh_private.param<bool>("switch_active_mode", switch_active_mode, false);

  dynamic_reconfigure::Server<pavo_ros::pavo_ros_cfgConfig> server;
  dynamic_reconfigure::Server<pavo_ros::pavo_ros_cfgConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::Publisher scan_pub = nh_private.advertise<sensor_msgs::LaserScan>(scan_topic, 1000);

  std::vector<pavo_response_scan_t> scan_vec;
  if (!switch_active_mode) 
  {
        drv = new pavo::pavo_driver();
  } 
  else 
  {
        drv = new pavo::pavo_driver(host_ip, host_port);
  }
  drv->pavo_open(lidar_ip, lidar_port);
  ros::Time start_scan_time;
  ros::Time end_scan_time;
  double scan_duration;
  drv->enable_motor(enable_motor);

  if (!enable_motor) 
  {
    pid_t kill_num;
    kill_num = getppid();
    kill(kill_num, 4);
    return 0;
  }

  if (method == 0 || method == 1 || method == 2 || method == 3) {
    drv->enable_tail_filter(method);
    if (method > 0)
      ROS_INFO("success to eliminate the tail by using method: %d",
               (int)method);
  } else {
    ROS_ERROR("false to set tail filter!");
    return 0;
  }

  if(drv->get_motor_speed(motor_speed_get))
  {
	ROS_INFO("success to get speed!");
    ROS_INFO("the motor_speed: %d", motor_speed_get);
  }
  else{
	ROS_ERROR("false to get speed!");
    return -1;
  }
  
  if(motor_speed!=motor_speed_get)
  {
	if (drv->set_motor_speed(motor_speed)) {
       ROS_INFO("success to set speed!");
       ROS_INFO("motor_speed: %d", motor_speed);
	   sleep(15);
    } else {
       ROS_ERROR("false to set speed!");
      return -1;
    }
  }
  else{
	  ROS_INFO("success to set speed!");
      ROS_INFO("motor_speed: %d", motor_speed);
  }

  if (drv->set_merge_coef(merge_coef)) {
    ROS_INFO("success to set merge!");
    ROS_INFO("merge_coef: %d", merge_coef);
  } else {
    ROS_ERROR("false to set merge!");
    return -1;
  }

  ros::Rate rate(motor_speed);
  int count = 0;

  while (ros::ok()) 
  {
    start_scan_time = ros::Time::now();
    bool status = drv->get_scanned_data(scan_vec,150);
    if(!status)
    {   delete drv;
        if (!switch_active_mode) 
        {
            drv = new pavo::pavo_driver();
        } 
        else 
        {
            drv = new pavo::pavo_driver(host_ip, host_port);
        }
            drv->pavo_open(lidar_ip, lidar_port);
    }
    count = scan_vec.size();
    end_scan_time = ros::Time::now();
    scan_duration = (end_scan_time - start_scan_time).toSec();
    nh.getParam("angle_min", angle_min);
    nh.getParam("angle_max", angle_max);
    nh.getParam("range_min", min_range);
    nh.getParam("range_max", max_range);
    nh.getParam("frame_id", frame_id);
    nh.getParam("inverted", inverted);
    nh.getParam("method", method);
    publish_msg(&scan_pub, scan_vec, start_scan_time, scan_duration, frame_id,
                inverted, angle_min, angle_max, min_range, max_range, method,
                switch_active_mode);
    ros::spinOnce();
    rate.sleep();
  }
    delete drv;
}
