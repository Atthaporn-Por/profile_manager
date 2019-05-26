#include <profile_manager/profile_manager_ros.h>

ProfileManagerRos::ProfileManagerRos(ros::NodeHandle &n){
    //implement here
}

void ProfileManagerRos::peopleCb(const skeleton_msgs::People::ConstPtr &p){
    //implement here
}   

void  ProfileManagerRos::dbChangeCb(const dinsow_msgs::DBChange::ConstPtr &p){
    //implement here
}

void ProfileManagerRos::publish(const dinsow_msgs::DinsowAction &action){
    //implement here
}

void ProfileManagerRos::publish(const std::string &s){
    //implement here
}

void ProfileManagerRos::publish(const geometry_msgs::Point &p){
    //implement here
}

void ProfileManagerRos::run(){
    //implement here
}

void ProfileManagerRos::callDinsowConfig(){
    //implement here
}