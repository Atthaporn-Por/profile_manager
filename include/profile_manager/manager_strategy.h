#ifndef MANAGER_STRATEGY_H
#define MANAGER_STRATEGY_H

#include <profile_manager/state.h>
#include <profile_manager/config.h>
#include <geometry_msgs/Point.h>

class ManagerStrategy{
    private:
        State* state;
        Config* config;
        geometry_msgs::Point focus_point;
    public:
        ManagerStrategy(State &state, Config& config);
        geometry_msgs::Point getFocusPoint();
        
        void setState(State state);
        void setFocusPoint(const geometry_msgs::Point &p);
        int getCurrentState();
        std::string getCurrentMode();
};

#endif