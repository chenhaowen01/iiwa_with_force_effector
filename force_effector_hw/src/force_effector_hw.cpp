#include <force_effector_hw.h>

ForceEffectorHW::ForceEffectorHW(ros::NodeHandle nh) : 
    nh_(nh)
{
}

void ForceEffectorHW::start()
{
    nh_.param("ft_sensor_topic_name", ft_sensor_name_, std::string("ft_sensor_topic"));
    ros::NodeHandle n;
    subscriber_ = n.subscribe(ft_sensor_name_, 1, &ForceEffectorHW::ft_sensor_subscriber_callback, this);

    nh_.param("joint_1_name", joint_1_name_, std::string("force_effector_joint_1"));
    nh_.param("ft_sensor_name", ft_sensor_name_, std::string("ft_sensor"));
    nh_.param("ft_sensor_frame_id", ft_sensor_frame_id_, std::string("ft_sensor_link"));

    hardware_interface::JointStateHandle state_handle(joint_1_name_, &joint_1_position_, &joint_1_velocity_, &joint_1_effort_);
    state_interface_.registerHandle(state_handle);
    this->registerInterface(&state_interface_);

    hardware_interface::JointHandle joint_handle(state_handle, &joint_1_effort_command_);
    effort_interface_.registerHandle(joint_handle);
    this->registerInterface(&effort_interface_);

    hardware_interface::ForceTorqueSensorHandle ft_sensor_handle(ft_sensor_name_, ft_sensor_frame_id_, force_, torque_);
    ft_interface_.registerHandle(ft_sensor_handle);
    this->registerInterface(&ft_interface_);
}

void ForceEffectorHW::read(const ros::Time &time, const ros::Duration &period)
{
}

void ForceEffectorHW::write(const ros::Time &time, const ros::Duration &period)
{
}

void ForceEffectorHW::ft_sensor_subscriber_callback(const geometry_msgs::WrenchStampedConstPtr &ft)
{
    force_[0] = ft->wrench.force.x;
    force_[1] = ft->wrench.force.y;
    force_[2] = ft->wrench.force.z;
    torque_[0] = ft->wrench.torque.x;
    torque_[1] = ft->wrench.torque.y;
    torque_[2] = ft->wrench.torque.z;
}