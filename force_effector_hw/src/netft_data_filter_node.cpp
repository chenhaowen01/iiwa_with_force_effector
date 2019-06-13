#include <ros/ros.h>
#include <lpfilter.h>
#include <signal.h>
#include <geometry_msgs/WrenchStamped.h>

bool quit = false;
void quitRequested(int sig) { quit = true; }

std::vector<double> g_raw_ft_data(6);

void netft_data_subscriber_callback(const geometry_msgs::WrenchStampedConstPtr &ft);
void lp_filter(std::vector<double> in, std::vector<double> &out);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "netft_data_filter_node", ros::init_options::NoSigintHandler);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);

    ros::NodeHandle n;
    ros::Subscriber subscriber = n.subscribe<geometry_msgs::WrenchStamped>("netft_data", 1, netft_data_subscriber_callback);
    ros::Publisher netft_data_filtered_publisher = n.advertise<geometry_msgs::WrenchStamped>("netft_data_filtered", 1);

    double freq =500;
    ros::Rate rate(freq);

    LPFilter filter(5/freq, 50, 6);

    ROS_INFO("starting...");

    std::vector<double> filtered_ft_data(6);
    geometry_msgs::WrenchStamped filtered_ft_msg;
    while(!quit){
        ROS_INFO("zz: %lf", g_raw_ft_data[2]);
        // filter.update(g_raw_ft_data, filtered_ft_data);
        lp_filter(g_raw_ft_data, filtered_ft_data);
        ROS_INFO("ff: %lf", filtered_ft_data[2]);
        filtered_ft_msg.header.seq += 1;
        filtered_ft_msg.header.stamp = ros::Time::now();
        filtered_ft_msg.wrench.force.x = filtered_ft_data[0];
        filtered_ft_msg.wrench.force.y = filtered_ft_data[1];
        filtered_ft_msg.wrench.force.z = filtered_ft_data[2];
        filtered_ft_msg.wrench.torque.x = filtered_ft_data[3];
        filtered_ft_msg.wrench.torque.y = filtered_ft_data[4];
        filtered_ft_msg.wrench.torque.z = filtered_ft_data[5];
        netft_data_filtered_publisher.publish(filtered_ft_msg);
        rate.sleep();
    }

    spinner.stop();
    return 0;
}

void netft_data_subscriber_callback(const geometry_msgs::WrenchStampedConstPtr &ft)
{
    ROS_INFO("fz: %lf", ft->wrench.force.z);
    g_raw_ft_data[0]= ft->wrench.force.x;
    g_raw_ft_data[1]= ft->wrench.force.y;
    g_raw_ft_data[2]= ft->wrench.force.z;
    g_raw_ft_data[3] = ft->wrench.torque.x;
    g_raw_ft_data[4] = ft->wrench.torque.y;
    g_raw_ft_data[5] = ft->wrench.torque.z;
}

void lp_filter(std::vector<double> in, std::vector<double> &out) 
{
    static double wt = 2 * 3.14159265 * 20 * 0.002;
    static double den = 1 + wt*wt;
    static double a = 1.0 / den;
    static double b = -2.0 / den;
    static double c = -wt*wt / den;

    static std::vector<double> out_1(6);
    static std::vector<double> out_2(6);

    for (int i=0; i<6; i++) {
        out[i] = -a*out_2[i] + -b*out_1[i] + -c*in[i];
    }

    out_2.assign(out_1.begin(), out_1.end());
    out_1.assign(out.begin(), out.end());
}