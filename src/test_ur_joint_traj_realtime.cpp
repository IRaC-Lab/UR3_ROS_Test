#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <cstdlib>

std::vector<double> current_position(6, 0.0);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

    const std::vector<std::string> ur5_joint_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    for(int i = 0; i <ur5_joint_names.size(); i++)
    {
        for (int j = 0; j < ur5_joint_names.size(); j++)
        {
            if(msg->name[j] == ur5_joint_names[i])
            {
                current_position[i] = msg->position[j];
                break;
            }
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ur_joint_traj_realtime_node");
    ros::NodeHandle nh;

    ros::Publisher joint_traj_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command",10);
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 10, jointStateCallback);


    while (joint_traj_publisher.getNumSubscribers() == 0 && ros::ok()) {
        ROS_INFO("Waiting for subscribers...");
        ros::Duration(0.5).sleep();
    }

    while (joint_state_sub.getNumPublishers() == 0 && ros::ok())
    {
        ROS_INFO("Waithing for joint states...");
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    
    const double step_duration = 0.5;
    const double home_duration = 3.0;

    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint target_point;

    traj.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    
    traj.header.stamp = ros::Time::now();
    target_point.positions.resize(6, 0.0);
    target_point.time_from_start = ros::Duration(home_duration);  

    traj.points.push_back(target_point);
    joint_traj_publisher.publish(traj);
    ros::Duration(home_duration).sleep();

    ROS_INFO("Moving to home posotion...");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ROS_INFO("Initial position[1] : %f", current_position[1]);
    

    for (int i = 0; i < 7; ++i)
    {
        traj.points.clear();

        traj.header.stamp = ros::Time::now();

        target_point.positions = current_position;
        target_point.positions[1] += -0.1;
        target_point.time_from_start = ros::Duration(step_duration);
        
        traj.points.push_back(target_point);

        joint_traj_publisher.publish(traj);
        
        ROS_INFO("Step %d: Moving to position[1] = %f", i + 1, target_point.positions[1]);

        ros::Duration(step_duration).sleep();
        ros::spinOnce();
        ROS_INFO("current position[1] : %f", current_position[1]);
    }
    

    ROS_INFO("Trjaectory execution completed");
    
    return 0;
}