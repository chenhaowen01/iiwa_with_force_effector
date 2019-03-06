#include <iiwa_ros/state/destination_reached.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>
#include <iiwa_ros/service/path_parameters.hpp>
#include <iiwa_ros/service/path_parameters_lin.hpp>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/CartesianPose.h>
#include <iiwa_msgs/MoveToCartesianPoseAction.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Time.h>
#include <actionlib/client/simple_action_client.h>

#define ROBOT_NAME "iiwa"

bool limitVelocityAndAcceleration(double joint_relative_velocity,
                                  double joint_relative_acceleration,
                                  double max_cartesian_velocity,
                                  double max_rotate_speed)
{
    iiwa_ros::service::PathParametersService pathParametersService;
    pathParametersService.init(ROBOT_NAME);
    iiwa_ros::service::PathParametersLinService pathParametersLinService;
    pathParametersLinService.init(ROBOT_NAME);

    pathParametersLinService.setVerbosity(true);

    bool ret = false;

    if (pathParametersService.setPathParameters(joint_relative_velocity, joint_relative_acceleration)) {
        ret = true;
    }

    geometry_msgs::Twist twist;
    twist.linear.x = max_cartesian_velocity;
    twist.linear.y = max_cartesian_velocity;
    twist.linear.z = max_cartesian_velocity;
    twist.angular.x = max_rotate_speed;
    twist.angular.y = max_rotate_speed;
    twist.angular.z = max_rotate_speed;


    
    if (pathParametersLinService.setMaxCartesianVelocity(twist)) {
        ret = true;
    }

    return ret;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_move_to_cartesian_pose_lin");

    ROS_INFO("ROS inited!");

    if (!limitVelocityAndAcceleration(0.01, 0.1, 0.04, 1)) {
        ROS_INFO("set velocity and acceleration failed!");
        return 1;
    }

    iiwa_ros::command::JointPosition jointPositionCommand;
    jointPositionCommand.init(ROBOT_NAME);
    if (!jointPositionCommand.isConnected()) {
        ROS_INFO("connect failed!");
        return 1;
    }

    iiwa_msgs::JointPosition readyPosition;
    readyPosition.header.seq = 1;
    readyPosition.header.stamp = ros::Time::now();
    // ready pose
    readyPosition.position.a1 = 0;
    readyPosition.position.a2 = 1.0472;
    readyPosition.position.a3 = 0;
    readyPosition.position.a4 = -1.0472;
    readyPosition.position.a5 = 0;
    readyPosition.position.a6 = -0.5236;
    readyPosition.position.a7 = 0;
    jointPositionCommand.setPosition(readyPosition);

    ros::Duration(1).sleep();

    // // wait to reach destination
    // iiwa_ros::service::TimeToDestinationService timeToDestinationService;
    // while(timeToDestinationService.getTimeToDestination() > 0){
    //     ros::Duration(0.1).sleep();
    // }

    std_msgs::Time timeToDestination;
    iiwa_ros::state::DestinationReached destinationReached;
    destinationReached.init(ROBOT_NAME);
    while(destinationReached.getTime().data.toSec() > 0){
        ros::Duration(0.1).sleep();
    }
    
    iiwa_ros::state::CartesianPose cartesianPoseState;
    cartesianPoseState.init(ROBOT_NAME);
    iiwa_msgs::CartesianPose readyPose = cartesianPoseState.getPose();

    actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> actionClient(ROBOT_NAME"/action/move_to_cartestian_pose_lin", true);

    actionClient.waitForServer();

    iiwa_msgs::CartesianPose targetPose(readyPose);
    targetPose.poseStamped.header.seq = 1;
    targetPose.poseStamped.header.stamp = ros::Time::now();
    targetPose.poseStamped.pose.position.x -= 0.1;
    
    iiwa_msgs::MoveToCartesianPoseGoal targetPoseGoal;
    targetPoseGoal.cartesian_pose = targetPose;

    actionClient.sendGoalAndWait(targetPoseGoal);

    targetPose.poseStamped.header.seq += 1;
    targetPose.poseStamped.header.stamp = ros::Time::now();
    targetPose.poseStamped.pose.position.x += 0.1;
    
    targetPoseGoal.cartesian_pose = targetPose;
    actionClient.sendGoalAndWait(targetPoseGoal);
    
    return 0;
}

