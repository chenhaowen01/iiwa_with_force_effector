#ifndef FORCE_EFFECTOR_FORCE_CONTROLLER_H
#define FORCE_EFFECTOR_FORCE_CONTROLLER_H

#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_publisher.h>

namespace force_effector_controllers
{

class ForceEffectorForceController: public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, hardware_interface::ForceTorqueSensorInterface>
{
public:
    ForceEffectorForceController();
    ~ForceEffectorForceController();

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);

    double command_;

private:
    hardware_interface::JointHandle joint_;
    hardware_interface::ForceTorqueSensorHandle ft_sensor_;
    int loop_count_;
    control_toolbox::Pid pid_controller_;

    boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      control_msgs::JointControllerState> > controller_state_publisher_ ;

    ros::Subscriber sub_command_;
    
    void setCommandCB(const std_msgs::Float64ConstPtr& msg);
};

}

#endif