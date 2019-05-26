#include <profile_manager/config.h>

Config::Config(){
    //implement here
}

Config::Config(ros::NodeHandle &n){
    //implement here
}

dinsow_msgs::DinsowAction Config::getAction(actionState state){
    //implement here
}

std::string Config::resolveTopic(const std::string &topic){
    //implement here
}

std::string Config::getBaseFrame(){
    //implement here
}

double Config::getFrequenzy(){
    //implement here
}