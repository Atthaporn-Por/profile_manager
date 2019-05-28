#ifndef MANAGER_STRATEGY_H
#define MANAGER_STRATEGY_H

#include <profile_manager/state.h>
#include <profile_manager/config.h>
#include <geometry_msgs/Point.h>

class State;

class ManagerStrategy{
    private:
        State* state;
        Config* config;
        geometry_msgs::Point focus_point;
        bool do_action;
    public:
        ManagerStrategy(State &state, Config& config);
        geometry_msgs::Point getFocusPoint();
        
        void setState(State *state);
        void setFocusPoint(const geometry_msgs::Point &p);
        int getCurrentState();
        std::string getCurrentStateName();
        bool doAction();
        std::string getCurrentMode();
        dinsow_msgs::DinsowAction getAction();
};

#endif