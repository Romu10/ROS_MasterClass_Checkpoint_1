#include "ros/ros.h"
#include "my_rb1_ros/Rotate.h"  
#include "ros/subscriber.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ostream>
#include <std_msgs/Float32.h>
#include <iostream>

class MoveRB1
{
  
    public:

        // ROS Objects
        ros::NodeHandle nh_;

        // ROS Services
        ros::ServiceServer service;

        // ROS Publishers
        ros::Publisher vel_pub;

        // ROS Subscribers
        ros::Subscriber degrees_rotated;
    
        // ROS Messages
        geometry_msgs::Twist vel_msg;

        // Variables
        float current_theta = 0.0;
        
  
        MoveRB1()
        {
            service = nh_.advertiseService("/rotate_robot", &MoveRB1::my_callback, this);
            ROS_INFO("The Service /rotate_robot is READY");
            vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            degrees_rotated = nh_.subscribe("/odom", 10, &MoveRB1::odomCallback, this);
            
        }

        void rotate_robot(float vel)
        {
            vel_msg.angular.z = vel;
            vel_pub.publish(vel_msg);
        }
        
        bool my_callback(my_rb1_ros::Rotate::Request &req,
                        my_rb1_ros::Rotate::Response &res)
        {
            ROS_INFO("The Service /move_bb8_in_circle has been called");

            // Obtiene el número de grados desde la solicitud
            float degrees = req.degrees;
            float radians = degrees * (M_PI/180);
            float initial_theta = current_theta;

            // Calculus
            float target_theta = initial_theta + radians; 
            float error_percet = 0.05; // 5%
            float max_value_degree = target_theta+(target_theta*error_percet);
            float min_value_degree =  target_theta-(target_theta*error_percet);

            ros::Rate rate(10);  // Control loop rate

            while (ros::ok())
            {
                ROS_INFO("Current theta: %f | Target theta: %f | Initial theta: %f | Min_value: %f | Max_value: %f ", current_theta, target_theta, initial_theta, min_value_degree, max_value_degree);
                float orientation_error = target_theta - current_theta;
                if (fabs(orientation_error) < error_percet)
                {
                    rotate_robot(0.0);
                    float current_theta_degree = ((current_theta*180)/M_PI);
                    float target_theta_degree = ((target_theta*180)/M_PI);
                    ROS_INFO("Current theta in degree: %f ||| Target theta in degree: %f", current_theta_degree, target_theta_degree); 
                    ROS_INFO("Current theta equal to Target theta");
                    res.result = "Rotation completed successfully";
                    return true;
                }    

                if (orientation_error > 0.0)
                {
                    rotate_robot(0.1);
                }
                else 
                {
                    rotate_robot(-0.1);
                }
                
                ros::spinOnce();
                rate.sleep();
            }
           
            res.result = "Rotation NOT complete";
            ROS_INFO("Finished service /move_bb8_in_circle");
            return true;
        }

        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            current_theta = msg->pose.pose.orientation.z;
            // ROS_INFO("Received Odometry: z = %f", current_theta);
        }
    
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotate_robot_node");

  MoveRB1 moveRB1;
  ros::spin();

  return 0;
}
