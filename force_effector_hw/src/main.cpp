#include <force_effector_hw.h>
#include <ros/ros.h>
#include <signal.h>

bool quit = false;
void quitRequested(int sig) { quit = true; }

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "force_effector_hw", ros::init_options::NoSigintHandler);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    signal(SIGTERM, quitRequested);
    signal(SIGINT, quitRequested);
    signal(SIGHUP, quitRequested);

    ros::NodeHandle force_effector_nh;
    ForceEffectorHW force_effector_hw(force_effector_nh);
    force_effector_hw.start();

    ros::Time last(ros::Time::now());
    ros::Time now;
    ros::Duration period(1.0);

    controller_manager::ControllerManager manager(&force_effector_hw, force_effector_nh);

    while(!quit){
        now = ros::Time::now();
        period = now - last;
        last = now;

        force_effector_hw.read(now, period);
        manager.update(now, period);
        force_effector_hw.write(now, period);
    }

    spinner.stop();
    return 0;
}
