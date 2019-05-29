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
    if(ros::Time::now() - before < ros::Duration(timeout[state])){
        return;
    }
    switch (state)
    {
        case IDLE_STATE:
            manager->setState(new IdleState(profiles, distance, timeout));
            break;
        case STATE_1:
            manager->setState(new FirstState(profiles, distance, timeout));
            break;
        case STATE_2:
            manager->setState(new SecondState(profiles, distance, timeout));
            break;
        case STATE_3:
            manager->setState(new ThirdState(profiles, distance, timeout));
            break;
        case FOLLOWING_STATE:
            manager->setState(new FollowingState(profiles, distance, timeout));
            break;
        case END_STATE:
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
    if(!profiles->isEmpty()){
        double dist = profiles->getNearestDistance();

        if(dist < distance[STATE_1]){
            changeState(manager, STATE_1);
        }else if(dist < distance[STATE_2]){
            changeState(manager, STATE_2);
        }else if(dist < distance[STATE_3]){
            changeState(manager, STATE_3);
        }else if(dist < distance[FOLLOWING_STATE]){
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
    //implement here
    if(first_call){

    }else{

    }
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
    //implement here
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
    //implement here
}

int ThirdState::getState(){
    return STATE_3;
}

std::string ThirdState::getStateName(){
    return "STATE_3";
}

/*---------------------------- FourthState ---------------------------*/
FollowingState::FollowingState(Profiles* profiles, double *distance, double *timeout) 
    : State(profiles, distance, timeout) {
        first_call = true;
}

void FollowingState::execute(ManagerStrategy *manager){
    if(important_id == -STATE_3){
        if(profiles->wasAll(*this)){
            manager->setFocusPoint(profiles->getNearestPoint());
        }else{
            manager->setFocusPoint(profiles->getNearestPoint(*this, distance[STATE_3]));
            changeState(manager, STATE_3);
        }
    }else if(important_id == -STATE_2){
        if(profiles->wasAll(*this)){
            manager->setFocusPoint(profiles->getNearestPoint());
        }else{
            manager->setFocusPoint(profiles->getNearestPoint(*this, distance[STATE_2]));
            changeState(manager, STATE_2);
        }
    }else{
        manager->setFocusPoint(profiles->getFocusPoint(important_id));
        if(profiles->distanceOf(profiles->getFocusPoint(important_id)) > distance[STATE_1]){
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
}

int EndState::getState(){
    return END_STATE;
}

std::string EndState::getStateName(){
    return "END_STATE";
}