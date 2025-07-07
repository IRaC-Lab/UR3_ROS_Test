#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <string>
#include <vector>

namespace joint_trajectory_client
{
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

    void doneCallback(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
        ROS_INFO("Trajectory execution finished in state [%s]", state.toString().c_str());
        if (result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
        {
            ROS_INFO("Trajectory executed successfully!");
        }
        else {
            ROS_WARN("Trajectory execution failed with error code: %d", result->error_code);
        }
    }

    void feedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback) {

    }


    class JointTrajectoryActionClient {
        public:
        JointTrajectoryActionClient();
        ~JointTrajectoryActionClient();

        bool sendTrajectory(const std::vector<double>& positions, double duration);

        actionlib::SimpleClientGoalState GetState() const;

        bool waitForResult(double timeout = 30.0);

        void cancelGoal();

        private:
        TrajectoryClient* trajectory_client_;
        std::vector<std::string> joint_names_;

        bool validatePositionSize(const std::vector<double>& positions) const;

        void setTrajectoryHeader(control_msgs::FollowJointTrajectoryGoal* goal) const;
    };

    JointTrajectoryActionClient::JointTrajectoryActionClient()
        : trajectory_client_(nullptr) {
        trajectory_client_ = new TrajectoryClient("/eff_joint_traj_controller/follow_joint_trajectory", true);

        joint_names_.reserve(6);
        joint_names_.push_back("shoulder_pan_joint");
        joint_names_.push_back("shoulder_lift_joint");
        joint_names_.push_back("elbow_joint");
        joint_names_.push_back("wrist_1_joint");
        joint_names_.push_back("wrist_2_joint");
        joint_names_.push_back("wrist_3_joint");

        ROS_INFO("Waiting for action server to start...");
        trajectory_client_->waitForServer();
        ROS_INFO("Action server started");
    }

    JointTrajectoryActionClient::~JointTrajectoryActionClient() {
        delete trajectory_client_;
    }

    bool JointTrajectoryActionClient::sendTrajectory(const std::vector<double>& positions, double duration){
        if (!validatePositionSize(positions)){
            return false;
        }

        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names = joint_names_;
        
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start = ros::Duration(duration);

        goal.trajectory.points.push_back(point);
        setTrajectoryHeader(&goal);

        // 동기 방식
        trajectory_client_->sendGoal(goal);

        // 비동기 방식일때 callback 등록
        //trajectory_client_->sendGoal(goal,&doneCallback);

        return true;
    }

    actionlib::SimpleClientGoalState JointTrajectoryActionClient::GetState() const {
        return trajectory_client_->getState();
    }

    bool JointTrajectoryActionClient::waitForResult(double timeout) {
        return trajectory_client_->waitForResult(ros::Duration(timeout));
    }

    void JointTrajectoryActionClient::cancelGoal() {
        trajectory_client_->cancelGoal();
    }

    bool JointTrajectoryActionClient::validatePositionSize(const std::vector<double>& positions) const {
        if (positions.size() != joint_names_.size()) {
            ROS_ERROR("Position vector size (%zu) mismatch with joint names size (%zu)", positions.size(), joint_names_.size());
            return false;
        }
        return true;
    }

    void JointTrajectoryActionClient::setTrajectoryHeader(control_msgs::FollowJointTrajectoryGoal* goal) const {
        goal->trajectory.header.stamp = ros::Time::now();
    }

    bool executeSinglePointTrajectory(JointTrajectoryActionClient* client) {
        //const std::vector<double> kTargetPositions = {0.5, -1.0, -0.5, 0.0, 1.5, 0.0};
        const std::vector<double> kTargetPositions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        const double kExecutionTime = 3.0;
        const double kTimeout = 10.0;

        ROS_INFO("Sending single point trajectory...");
        if (!client->sendTrajectory(kTargetPositions, kExecutionTime))
        {
            ROS_ERROR("Failed to send single point trajectory");
            return false;
        }

        //========= 동기 방식(비동기 방식일때는 삭제)=========================================
        if (!client->waitForResult(kTimeout))
        {
            ROS_ERROR("Trajectory execution timeout");
            client->cancelGoal();
            return false;
        }

        if (client->GetState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Single point trajectort completed successfully!");
            return true;
        }
        else
        {
            ROS_WARN("Single point trajectort failed: %s", client->GetState().toString().c_str());
            return false;
        }
        //===============================================================================
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_action_client");

    joint_trajectory_client::JointTrajectoryActionClient client;

    const double kWaitTime = 2.0;

    joint_trajectory_client::executeSinglePointTrajectory(&client);

    // 비동기 방식일때 사용(callback 함수 확인용)
    //ros::spin();

    ROS_INFO("All trajectories completed. Node shutting down");
    return 0;
}

