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
        float initial_theta = 0.0;
        float target_theta = 0.0;
        bool ready = false;
        bool completed = false;
  
        MoveRB1()
        {
            service = nh_.advertiseService("/rotate_robot", &MoveRB1::my_callback, this);
            ROS_INFO("The Service /rotate_robot is READY");
            vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
            degrees_rotated = nh_.subscribe("/odom", 20, &MoveRB1::odomCallback, this);
            // loop_rate_ = ros::Rate(10);
        }
        
        void rotate_robot_left()
        {
            vel_msg.angular.z = 0.1;
            vel_pub.publish(vel_msg);
        }

        void rotate_robot_right()
        {
            vel_msg.angular.z = -0.1;
            vel_pub.publish(vel_msg);
        }

        void stop_robot()
        {
            vel_msg.angular.z = 0.0;
            vel_pub.publish(vel_msg);
        }
        
        bool my_callback(my_rb1_ros::Rotate::Request &req,
                        my_rb1_ros::Rotate::Response &res)
        {
            ROS_INFO("The Service /move_bb8_in_circle has been called");

            // Obtiene el número de grados desde la solicitud
            float degrees = req.degrees;
            float radians = degrees * (M_PI/180);
            std::cout << "The robot will rotate: " << degrees << "Degree" << std::endl;

            if (initial_theta == 0)
            {
                initial_theta = current_theta;
                ROS_INFO("Initial thehta is: %f", initial_theta);
            }

            // Calculus
            target_theta = (degrees/100) + initial_theta; 

            ready = true;
            
            if (completed)
            {
                // Indica si la rotación se ha completado con éxito o no en la respuesta
                res.result = "Rotation completed successfully";
            }
           
            
            ROS_INFO("Finished service /move_bb8_in_circle");
            return true;
        }

        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            current_theta = msg->pose.pose.orientation.z;
            // ROS_INFO("Received Odometry: z = %f", current_theta);

            // Start the rotation of the robot
            if (ready)
            {
                if (target_theta > initial_theta)
                {  
                    current_theta = msg->pose.pose.orientation.z;
                    rotate_robot_left();
                    ROS_INFO("Current theta: %f | Target theta: %f | initial_theta: %f", current_theta, target_theta, initial_theta);

                    if (std::abs((target_theta+(target_theta*0.1))) >= std::abs(current_theta) && target_theta != 0)
                    {
                        current_theta = msg->pose.pose.orientation.z;
                        ROS_INFO("Current theta: %f ||| Target theta: %f", current_theta, target_theta); 
                        stop_robot();
                        ready = false;
                        completed = true;
                        ROS_INFO("Current theta equal to Target theta");
                    }   
                }
                
                if (target_theta <= current_theta)
                {
                    current_theta = msg->pose.pose.orientation.z;
                    rotate_robot_right();
                    ROS_INFO("Current theta: %f || Target theta: %f", current_theta, target_theta); 

                    if (std::abs((target_theta+(target_theta*0.1))) >= std::abs(current_theta) && target_theta != 0)
                    {
                        current_theta = msg->pose.pose.orientation.z;
                        ROS_INFO("Current theta: %f ||| Target theta: %f", current_theta, target_theta); 
                        stop_robot();
                        ready = false;
                        completed = true;
                        ROS_INFO("Current theta equal to Target theta");
                    }
                }
                
            }
        }
    
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotate_robot_node");

  MoveRB1 moveRB1;
  ros::spin();

  return 0;
}
