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
    std::vector<std::string> ur5_joint_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };

    for(size_t i = 0; i <ur5_joint_names.size(); i++)
    {
        for (size_t j = 0; j < ur5_joint_names.size(); j++)
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
    ros::init(argc, argv, "test_ur_joint_trajectory_publisher_node");
    ros::NodeHandle node_obj;

    ros::Publisher joint_traj_publisher;
    ros::Subscriber joint_state_sub;

    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint target_point;
    trajectory_msgs::JointTrajectoryPoint start_point;
    trajectory_msgs::JointTrajectoryPoint home_point;

    joint_traj_publisher = node_obj.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command",10);

    joint_state_sub = node_obj.subscribe("/joint_states", 10, jointStateCallback);

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
    
    
    traj.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    target_point.positions.resize(6, 0.0);
    start_point.positions.resize(6, 0.0);
    home_point.positions.resize(6, 0.0);

    start_point.positions = current_position;
    start_point.velocities.resize(6, 0.0);
    start_point.time_from_start = ros::Duration(0.0);
    traj.points.push_back(start_point);

    home_point.time_from_start = ros::Duration(1.0);
    traj.points.push_back(home_point);

    traj.header.stamp = ros::Time::now();
    target_point.positions = current_position;
    target_point.positions[0] = 0.0;
    target_point.positions[2] = 0.0;
    target_point.positions[4] = 0.0;
    target_point.time_from_start = ros::Duration(3.0);
    
    traj.points.push_back(target_point);

    joint_traj_publisher.publish(traj);

    ros::Duration(1.0).sleep();

    ROS_INFO("Trjaectory published");
    
    return 0;
}