#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_moveit_custom_position");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(2.0).sleep();

    moveit::planning_interface::MoveGroupInterface group("manipulator"); //Group 이름은 srdf에서 확인
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    ROS_INFO("Reference frame: %s", group.getEndEffector().c_str());

    geometry_msgs::Pose target_pose;

    ros::Rate rate(10);
    int loop_num = 10;
    const double z_step = 0.01;
    const double z_upper = 0.5;
    const double z_lower = 0.1;

    while (ros::ok())
    {
        geometry_msgs::Pose current_pose = group.getCurrentPose().pose;
        target_pose = current_pose;
        ROS_INFO("Current z: %f", current_pose.position.z);

        target_pose.position.z += z_step;
        ROS_INFO("Target z: %f", target_pose.position.z);

        if (target_pose.position.z > z_upper)
        {
            ROS_WARN("Target z is above safe workspace. Stopping.");
            break;
        }
        if (target_pose.position.z < z_lower)
        {
            ROS_WARN("Target z is below safe workspace. Stopping.");
            break;
        }

        group.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            auto move_result = group.move();
            if (!move_result)
            {
                ROS_ERROR("Move execution failed!");
            }
            
        }
        else
        {
            ROS_WARN("Planning failed. Skipping this step.");
        }
        
        rate.sleep();
        loop_num -= 1;
        if (loop_num == 0)
        {
            ROS_INFO("Loop is finished!!");
            break;
        }
        
    }
    
    ros::shutdown();

}