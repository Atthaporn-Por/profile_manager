#ifndef CONFIG_H
#define CONFIG_H

#include <ros/ros.h>
#include <profile_manager/constant.h>
#include <dinsow_msgs/DinsowAction.h>

class Config{
    private:
        double frequenzy;
        std::string name_space;
        std::string node_name;
        std::string base_frame;
        dinsow_msgs::DinsowAction state_action[ACTION_SIZE];
    public:
        Config();
        Config(ros::NodeHandle &n);

        dinsow_msgs::DinsowAction getAction(actionState state);
        std::string resolveTopic(const std::string &topic);
        std::string getBaseFrame();
        double getFrequenzy();
};

#endif