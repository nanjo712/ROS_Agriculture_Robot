/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>
#include <chrono>
#include <tf/transform_broadcaster.h>
#include "utility.h"


namespace robomaster {


    class PIDPlanner {
    public:
        PIDPlanner(ros::NodeHandle &given_nh) : nh(given_nh), plan_(false) {

            nh.param<double>("max_x_speed", max_x_speed_, 1.0);
            nh.param<double>("max_y_speed", max_y_speed_, 1.0);
            nh.param<double>("max_yaw_speed", max_yaw_speed_, 2.0);
            nh.param<double>("p_x_coeff", p_x_coeff_, 1);
            nh.param<double>("p_y_coeff", p_y_coeff_, 1);
            nh.param<double>("p_yaw_coeff", p_yaw_coeff_, 1);
            nh.param<int>("plan_frequency", plan_freq_, 30);
            nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.1);
            nh.param<double>("goal_angle_tolerance", goal_angle_tolerance_, 0.05);

            tf_listener_ = std::make_shared<tf::TransformListener>();
            cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/calib_vel", 1);

            goal_sub_ = nh.subscribe("/vase_point", 1, &PIDPlanner::GoalCallback, this);
            plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_freq_), &PIDPlanner::Plan, this);
        }

        ~PIDPlanner() = default;

        void GoalCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
            goal_ = *msg;
            plan_ = true;

        }

    private:

        void Plan(const ros::TimerEvent &event) {

            if (plan_) {
                if (goal_.header.stamp-ros::Time::now() > ros::Duration(0.2))
                {
                    ROS_INFO("Goal is too old!");
                    return;
                }

                if (abs(goal_.point.x-0.5) <= goal_dist_tolerance_ ) {
                    plan_ = false;
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = 0;
                    cmd_vel.linear.z = 1; // Planning Success sig.2bringup
                    cmd_vel_pub_.publish(cmd_vel);
                    // ROS_INFO("Calib Success!");
                    return;
                }

                auto dx = goal_.point.x - 0.5;
                
                if (dx>0.1) dx=0.1;
                else if (dx<-0.1) dx=-0.1;

                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = -dx;
                cmd_vel_pub_.publish(cmd_vel);
                
            }
        }

    private:

        ros::NodeHandle nh;
        std::shared_ptr<tf::TransformListener> tf_listener_;

        geometry_msgs::PointStamped goal_;
        ros::Timer plan_timer_;

        ros::Subscriber goal_sub_;
        ros::Publisher cmd_vel_pub_;

        bool plan_;

        double max_x_speed_, max_y_speed_, max_yaw_speed_;
        double p_x_coeff_, p_y_coeff_, p_yaw_coeff_;
        double goal_dist_tolerance_, goal_angle_tolerance_;

        int plan_freq_;

    };
}
using namespace robomaster;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_planner");
  ros::NodeHandle nh("~");
  PIDPlanner pid_planner(nh);
  ros::spin();
  return 0;
}

