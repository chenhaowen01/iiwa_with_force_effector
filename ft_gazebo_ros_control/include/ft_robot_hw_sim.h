#ifndef FT_ROBOT_HW_SIM_H
#define FT_ROBOT_HW_SIM_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <geometry_msgs/WrenchStamped.h>

namespace gazebo_ros_control
{

class FtRobotHWSim : public DefaultRobotHWSim
{
public:
    virtual bool initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions);

protected:
    std::string ft_sensor_topic_name_;
    std::string ft_sensor_name_;
    std::string ft_sensor_frame_id_;
    double force_[3];
    double torque_[3];

    hardware_interface::ForceTorqueSensorInterface ft_interface_;

    ros::Subscriber subscriber_;
    void ft_sensor_subscriber_callback(const geometry_msgs::WrenchStampedConstPtr& ft);
};

typedef boost::shared_ptr<FtRobotHWSim> FtRobotHWSimPtr;

}

#endif