#include <profile_manager/state.h>

State::State(Profiles* profiles, double timeout){
    this->before = ros::Time::now();
    this->latest = ros::Time::now();
    this->profiles = profiles;
    this->timeout = ros::Duration(timeout);
    this->first_call = false;
}
/*---------------------------- IdleState ---------------------------*/
IdleState::IdleState(Profiles* profiles, double timeout) : State(profiles, timeout) {
    first_call = true;
}

void IdleState::execute(ManagerStrategy *manager){
    //implement here
    
}

int IdleState::getStateName(){
    return IDLE_STATE;
}

/*---------------------------- FirstState ---------------------------*/

FirstState::FirstState(Profiles* profiles, double timeout) : State(profiles, timeout){
    first_call = true;
}

void FirstState::execute(ManagerStrategy *manager){
    //implement here
}

int FirstState::getStateName(){
    return STATE_1;
}

/*---------------------------- SecondState ---------------------------*/
SecondState::SecondState(Profiles* profiles, double timeout) : State(profiles, timeout) {
    first_call = true;
}

void SecondState::execute(ManagerStrategy *manager){
    //implement here
}   

int SecondState::getStateName(){
    return STATE_2;
}

/*---------------------------- ThirdState ---------------------------*/
ThirdState::ThirdState(Profiles* profiles, double timeout) : State(profiles, timeout) {
    first_call = true;
} 

void ThirdState::execute(ManagerStrategy *manager){
    //implement here
}

int ThirdState::getStateName(){
    return STATE_3;
}

/*---------------------------- FourthState ---------------------------*/
FourthState::FourthState(Profiles* profiles, double timeout) : State(profiles, timeout) {
    first_call = true;
}

void FourthState::execute(ManagerStrategy *manager){
    //implement here
}

int FourthState::getStateName(){
    return STATE_4;
}

/*---------------------------- EndState ---------------------------*/
EndState::EndState(Profiles* profiles, double timeout) : State(profiles, timeout) {
    first_call = true;
} 

void EndState::execute(ManagerStrategy *manager){
    //implement here
}

int EndState::getStateName(){
    return END_STATE;
}