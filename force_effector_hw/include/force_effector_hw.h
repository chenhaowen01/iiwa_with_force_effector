#include <controller_manager/controller_manager.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/JointState.h>

class ForceEffectorHW : public hardware_interface::RobotHW
{
public:
    ForceEffectorHW(ros::NodeHandle nh);
    virtual ~ForceEffectorHW() = default;
    void start();
    void read(const ros::Time &time, const ros::Duration &period);
    void write(const ros::Time &time, const ros::Duration &period);

private:
    ros::NodeHandle nh_;

    std::string joint_1_name_;
    double joint_1_position_;
    double joint_1_velocity_;
    double joint_1_effort_;
    double joint_1_effort_command_;

    std::string joint_2_name_;

    std::string ft_sensor_topic_name_;
    std::string ft_sensor_name_;
    std::string ft_sensor_frame_id_;
    double force_[3];
    double torque_[3];

    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::EffortJointInterface effort_interface_; 
    hardware_interface::ForceTorqueSensorInterface ft_interface_;

    ros::Subscriber subscriber_;
    void ft_sensor_subscriber_callback(const geometry_msgs::WrenchStampedConstPtr &ft);

    ros::Publisher torque_publisher_;
    ros::Publisher joint_states_publisher_;
};