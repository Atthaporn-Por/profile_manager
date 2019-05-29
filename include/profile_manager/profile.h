#ifndef PROFILE_H
#define PROFILE_H

#include <ros/time.h>
#include <geometry_msgs/Point.h>
#include <profile_manager/constant.h>
#include <profile_manager/state.h>
#include <profile_manager/manager_strategy.h>
#include <stdexcept>

class ManagerStrategy;

struct Profile{
    const static int MAX_SIZE = 10;
    int id;
    bool isActive[ACTION_SIZE];
    bool wasActive[ACTION_SIZE];
    ros::Time first_time[ACTION_SIZE];
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
        void add(State *state, int id, const geometry_msgs::Point &p);
        void erase(int id);
        void setWasActive(State *state, int id);
        void setWasActive(State *state, std::vector<Profile>::iterator i);
        void setWasActiveAll(State *state);

        bool isEmpty();
        bool wasAll(State &state);
        int getNearestId();
        geometry_msgs::Point getFocusPoint(int id);
        geometry_msgs::Point getNearestPoint();
        geometry_msgs::Point getNearestPoint(State &state, double max_distance);
        double getNearestDistance();
        double distanceOf(const Profile &p);
        double distanceOf(const geometry_msgs::Point &p);

};

#endif