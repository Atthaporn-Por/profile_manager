#ifndef PROFILE_H
#define PROFILE_H

#include <ros/time.h>
#include <geometry_msgs/Point.h>
#include <profile_manager/constant.h>

struct Profile{
    const static int MAX_SIZE = 10;
    int id;
    bool isActive[MAX_SIZE];
    bool wasActive[MAX_SIZE];
    ros::Time first_time[MAX_SIZE];
    ros::Time seen_time;
    geometry_msgs::Point focus_point;

    Profile(int id, const geometry_msgs::Point &p){
        this->id = id;
        seen_time = ros::Time::now();
        for(int i = 0;i < ACTION_SIZE;i++){
            isActive[i] = false;
            wasActive[i] = false;
        }
        focus_point = p;
    }
};

class Profiles{
    private:
        std::vector<Profile> profiles;
    public:
        Profiles();
        void add(int id, const geometry_msgs::Point &p);
        void erase(int id);
};

#endif