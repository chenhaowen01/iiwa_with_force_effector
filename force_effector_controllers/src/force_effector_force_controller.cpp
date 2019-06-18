#include <force_effector_controllers/force_effector_force_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace force_effector_controllers
{

ForceEffectorForceController::ForceEffectorForceController()
    : command_(0), loop_count_(0)
{}

ForceEffectorForceController::~ForceEffectorForceController()
{
    sub_command_.shutdown();
}

bool ForceEffectorForceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n)
{
    std::string joint_name;
    if (!n.getParam("joint", joint_name))
    {
        ROS_ERROR("No joint giver (namespace: %s)", n.getNamespace().c_str());
        return false;
    }

    std::string ft_sensor_name;
    if (!n.getParam("ft_sensor", ft_sensor_name))
    {
        ROS_ERROR("No ft sensor giver (namespace: %s)", n.getNamespace().c_str());
        return false;
    }

    hardware_interface::EffortJointInterface *const effort_joint_hw = robot_hw->get<hardware_interface::EffortJointInterface>();
    hardware_interface::ForceTorqueSensorInterface *const ft_sensor_hw = robot_hw->get<hardware_interface::ForceTorqueSensorInterface>();

    joint_ = effort_joint_hw->getHandle(joint_name);
    ft_sensor_ = ft_sensor_hw->getHandle(ft_sensor_name);

    if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    {
        return false;
    }

    controller_state_publisher_.reset(
        new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1)
    );

    sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &ForceEffectorForceController::setCommandCB, this);

    return true;
}

void ForceEffectorForceController::starting(const ros::Time& time)
{
    command_ = 0.0;
    pid_controller_.reset();
}

void ForceEffectorForceController::update(const ros::Time& time, const ros::Duration& period)
{
    static double last_command = 0;
    // ROS_INFO("loop count: %d, %lf", loop_count_, ft_sensor_.getForce()[2]);
    double error = command_ - ft_sensor_.getForce()[0];

    // double commanded_force = pid_controller_.computeCommand(error, period);

    if (last_command != command_) {
        pid_controller_.reset();
    }

    double commanded_force = (command_ + 8) * -29;
    commanded_force += pid_controller_.computeCommand(error, period);

    joint_.setCommand(commanded_force);

    last_command = command_;

    if (loop_count_ % 10 == 0)
    {
        if (controller_state_publisher_ && controller_state_publisher_->trylock())
        {
            controller_state_publisher_->msg_.header.stamp = time;
            controller_state_publisher_->msg_.set_point = command_;
            controller_state_publisher_->msg_.process_value = ft_sensor_.getForce()[2];
            controller_state_publisher_->msg_.error = error;
            controller_state_publisher_->msg_.time_step = period.toSec();
            controller_state_publisher_->msg_.command = commanded_force;

            double dummy;
            bool antiwindup;
            pid_controller_.getGains(controller_state_publisher_->msg_.p,
                controller_state_publisher_->msg_.i,
                controller_state_publisher_->msg_.d,
                controller_state_publisher_->msg_.i_clamp,
                dummy,
                antiwindup);
            controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
            controller_state_publisher_->unlockAndPublish();
        }
    }

    loop_count_++;
}

void ForceEffectorForceController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
    command_ = msg->data;
}

}

PLUGINLIB_EXPORT_CLASS( force_effector_controllers::ForceEffectorForceController, controller_interface::ControllerBase)
