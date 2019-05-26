#ifndef MANAGER_STRATEGY_H
#define MANAGER_STRATEGY_H

#include <profile_manager/state.h>
#include <profile_manager/config.h>
#include <geometry_msgs/Point.h>

class ManagerStrategy{
    private:
        State* state;
        Config* config;
    public:
        ManagerStrategy(State &state, Config& config);
        geometry_msgs::Point getFocusPoint();
        
        std::string getCurrentState();
        std::string getCurrentMode();
};

#endif