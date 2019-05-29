#include <profile_manager/manager_strategy.h>

ManagerStrategy::ManagerStrategy(State &state, Config& config){
    //implement here
    this->state = &state;
    this->config = &config;
    do_action = false;
}

geometry_msgs::Point ManagerStrategy::getFocusPoint(){
    state->execute(this);
    return focus_point;
}

int ManagerStrategy::getCurrentState(){
    return this->state->getState();
}

std::string ManagerStrategy::getCurrentStateName(){
    return this->state->getStateName();
}

std::string ManagerStrategy::getCurrentMode(){
    //implement here
}

void ManagerStrategy::setState(State *state){
    this->state = state;
}

void ManagerStrategy::setFocusPoint(const geometry_msgs::Point &p){
    this->focus_point = p;
}

bool ManagerStrategy::doAction(){
    return do_action;
}

void ManagerStrategy::setDoAction(bool do_action){
    this->do_action = do_action;
}

dinsow_msgs::DinsowAction ManagerStrategy::getAction(){
    return config->getAction(state->getState());
}