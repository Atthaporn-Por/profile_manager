#include <ros/ros.h>
#include <profile_manager/profile_manager_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "profile_manager");
    ros::NodeHandle n;
    ProfileManagerRos node(n);
    ros::spin();
    return 0;
}