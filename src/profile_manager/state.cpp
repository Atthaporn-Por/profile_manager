#include <profile_manager/state.h>

State::State(Profiles* profiles, double *distance, double *timeout){
    this->before = ros::Time::now();
    this->latest = ros::Time::now();
    this->profiles = profiles;
    this->timeout = timeout;
    this->first_call = false;
    this->distance = distance;
    this->important_id = -STATE_3;
}

void State::changeState(ManagerStrategy *manager, int state){
    ROS_INFO("Count Donw : %f", (ros::Time::now() - before).toSec() );
    if(ros::Time::now() - before < ros::Duration(timeout[this->getState()]) && getState() != IDLE_STATE){
        return;
    }
    switch (state)
    {
        case IDLE_STATE:
            ROS_INFO("changeState : IDLE_STATE");
            manager->setState(new IdleState(profiles, distance, timeout));
            break;
        case STATE_1:
            if(!profiles->wasAll(*this)){
                ROS_INFO("changeState : FIRST_STATE");
                manager->setState(new FirstState(profiles, distance, timeout));
            }
            break;
        case STATE_2:
            if(!profiles->wasAll(*this)){
                ROS_INFO("changeState : SECOND_STATE");
                manager->setState(new SecondState(profiles, distance, timeout));
            }
            break;
        case STATE_3:
            if(!profiles->wasAll(*this)){
                ROS_INFO("changeState : THIRD_STATE");
                manager->setState(new ThirdState(profiles, distance, timeout));
            }
            break;
        case FOLLOWING_STATE:
            ROS_INFO("changeState : FOLLOWING_STATE");
            manager->setState(new FollowingState(profiles, distance, timeout));
            break;
        case END_STATE:
            ROS_INFO("changeState : END_STATE");
            manager->setState(new EndState(profiles, distance, timeout));
            break;
    }
}

/*---------------------------- IdleState ---------------------------*/
IdleState::IdleState(Profiles* profiles, double *distance, double *timeout) 
    : State(profiles, distance, timeout) {
        first_call = true;
}

void IdleState::execute(ManagerStrategy *manager){
    //implement here
    ROS_INFO("Idle_state is executed");
    if(!profiles->isEmpty()){
        double dist = profiles->getNearestDistance();
        ROS_INFO("dist in Idle_State : %f", sqrt(dist));
        if(sqrt(dist) < distance[STATE_1]){
            changeState(manager, STATE_1);
        }else if(sqrt(dist) < distance[STATE_2]){
            changeState(manager, STATE_2);
        }else if(sqrt(dist) < distance[STATE_3]){
            changeState(manager, STATE_3);
        }else if(sqrt(dist) < distance[FOLLOWING_STATE]){
            changeState(manager, FOLLOWING_STATE);
        }
    }else{
        before = ros::Time::now();
        latest = ros::Time::now();
    }
}

int IdleState::getState(){
    return IDLE_STATE;
}

std::string IdleState::getStateName(){
    return "IDLE_STATE";
}

/*---------------------------- FirstState ---------------------------*/
FirstState::FirstState(Profiles* profiles, double *distance, double *timeout) 
    : State(profiles, distance, timeout){
        first_call = true;
}

void FirstState::execute(ManagerStrategy *manager){
    if(first_call){
        if(!profiles->wasAll(*this)){
            manager->setDoAction(true);
        }
        important_id = profiles->getNearestId();
        first_call = false;
    }
    if(profiles->isEmpty()){
        changeState(manager, IDLE_STATE);
        return;
    }
    if(profiles->distanceOf(profiles->getFocusPoint(important_id)) > distance[STATE_1]){
        changeState(manager, IDLE_STATE);
        return;
    }
    if(profiles->wasAll(STATE_3)){
         changeState(manager, FOLLOWING_STATE);
    }
    profiles->setWasActive(this, important_id);
}

int FirstState::getState(){
    return STATE_1;
}

std::string FirstState::getStateName(){
    return "STATE_1";
}

/*---------------------------- SecondState ---------------------------*/
SecondState::SecondState(Profiles* profiles, double *distance, double *timeout) 
    : State(profiles, distance, timeout) {
        first_call = true;
}

void SecondState::execute(ManagerStrategy *manager){
    if(first_call){
        if(!profiles->wasAll(*this)){
            manager->setDoAction(true);
        }
        important_id = -STATE_2;
        first_call = false;
    }
    profiles->setWasActiveAll(this);
    if(profiles->isEmpty()){
        changeState(manager, IDLE_STATE);
    }else if(profiles->wasAll(STATE_2)){
        changeState(manager, FOLLOWING_STATE);
    }
}   

int SecondState::getState(){
    return STATE_2;
}

std::string SecondState::getStateName(){
    return "STATE_2";
}

/*---------------------------- ThirdState ---------------------------*/
ThirdState::ThirdState(Profiles* profiles, double *distance, double *timeout) 
    : State(profiles, distance, timeout) {
        first_call = true;
} 

void ThirdState::execute(ManagerStrategy *manager){
    ROS_INFO("ThirdState is executed");
    if(first_call){
        if(!profiles->wasAll(*this)){
            manager->setDoAction(true);
        }
        important_id = -STATE_3;
        first_call = false;
    }
    profiles->setWasActiveAll(this);
    if(profiles->isEmpty()){
        ROS_INFO("execute STATE_3 : changeState(IDLE_STATE)");
        changeState(manager, IDLE_STATE);
    }else if(profiles->wasAll(STATE_3)){
        ROS_INFO("execute STATE_3 : changeState(FOLLOWING_STATE)");
        changeState(manager, FOLLOWING_STATE);
    }
}

int ThirdState::getState(){
    return STATE_3;
}

std::string ThirdState::getStateName(){
    return "STATE_3";
}

/*---------------------------- FollowingState ---------------------------*/
FollowingState::FollowingState(Profiles* profiles, double *distance, double *timeout) 
    : State(profiles, distance, timeout) {
        first_call = true;
}

void FollowingState::execute(ManagerStrategy *manager){
    if(profiles->isEmpty()){
        changeState(manager, IDLE_STATE);
    }
    if(important_id == -STATE_3){
        if(!profiles->wasAll(STATE_3)){
            manager->setFocusPoint(profiles->getNearestPoint());
        }else{
            manager->setFocusPoint(profiles->getNearestPoint(*this, distance[STATE_3]));
            changeState(manager, STATE_3);
        }
    }else if(important_id == -STATE_2){
        if(!profiles->wasAll(STATE_2)){
            manager->setFocusPoint(profiles->getNearestPoint());
        }else{
            manager->setFocusPoint(profiles->getNearestPoint(*this, distance[STATE_2]));
            changeState(manager, STATE_2);
        }
    }else{
        manager->setFocusPoint(profiles->getFocusPoint(important_id));
        if(!profiles->distanceOf(profiles->getFocusPoint(important_id)) > distance[STATE_1]){
            changeState(manager, IDLE_STATE);
        }
    }    
}

int FollowingState::getState(){
    return FOLLOWING_STATE;
}

std::string FollowingState::getStateName(){
    return "FOLLOWING_STATE";
}

/*---------------------------- EndState ---------------------------*/
EndState::EndState(Profiles* profiles, double *distance, double *timeout) 
    : State(profiles, distance, timeout) {
        first_call = true;
} 

void EndState::execute(ManagerStrategy *manager){
    //implement here
    if(first_call){
        if(profiles->isEmpty()){
            manager->setDoAction(true);
        }
        first_call = false;
    }
    changeState(manager, IDLE_STATE);
}

int EndState::getState(){
    return END_STATE;
}

std::string EndState::getStateName(){
    return "END_STATE";
}