#include <ft_robot_hw_sim.h>

namespace gazebo_ros_control {

bool FtRobotHWSim::initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
{
    if (!DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions)){
        return false;
    }
    model_nh.param("ft_sensor_topic_name", ft_sensor_topic_name_, std::string("ft_sensor_topic"));

    ros::NodeHandle nh;
    subscriber_ = nh.subscribe(ft_sensor_topic_name_, 1, &FtRobotHWSim::ft_sensor_subscriber_callback, this);

    model_nh.param("ft_sensor_name", ft_sensor_name_, std::string("ft_sensor"));
    model_nh.param("ft_sensor_frame_id", ft_sensor_frame_id_, std::string("ft_sensor_link"));
    hardware_interface::ForceTorqueSensorHandle ft_sensor_handle(ft_sensor_name_, ft_sensor_frame_id_, force_, torque_);
    ft_interface_.registerHandle(ft_sensor_handle);
    this->registerInterface(&ft_interface_);
}

void FtRobotHWSim::ft_sensor_subscriber_callback(const geometry_msgs::WrenchStampedConstPtr& ft)
{
    force_[0] = ft->wrench.force.x;
    force_[1] = ft->wrench.force.y;
    force_[2] = ft->wrench.force.z;
    torque_[0] = ft->wrench.torque.x;
    torque_[1] = ft->wrench.torque.y;
    torque_[2] = ft->wrench.torque.z;
}

}

PLUGINLIB_EXPORT_CLASS( gazebo_ros_control::FtRobotHWSim, gazebo_ros_control::RobotHWSim)