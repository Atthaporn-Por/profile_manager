#include <profile_manager/state.h>

State::State(){
    this->before = ros::Time::now();
    this->latest = ros::Time::now();
    this->timeout = ros::Duration(2.0);
    this->first_call = false;
}

State::State(double timeout){
    this->before = ros::Time::now();
    this->latest = ros::Time::now();
    this->timeout = ros::Duration(timeout);
    this->first_call = false;
}
/*---------------------------- IdleState ---------------------------*/
IdleState::IdleState() : State() {
    first_call = true;
}

IdleState::IdleState(double timeout) : State(timeout) {
    first_call = true;
}

void IdleState::execute(ManagerStrategy *manager){
    //implement here
    if(first_call){
        if(ros::Time::now() - latest > ros::Duration(0.5)){
            this->first_call = false;
        }else if(ros::Time::now() - latest > timeout){
            this->first_call = false;
        }
    }
}

int IdleState::getStateName(){
    return IDLE_STATE;
}

/*---------------------------- FirstState ---------------------------*/
FirstState::FirstState() : State(){
    first_call = true;
}

FirstState::FirstState(double timeout) : State(timeout){
    first_call = true;
}

void FirstState::execute(ManagerStrategy *manager){
    //implement here
}

int FirstState::getStateName(){
    return STATE_1;
}

/*---------------------------- SecondState ---------------------------*/
SecondState::SecondState() : State() {
    first_call = true;
}

SecondState::SecondState(double timeout) : State(timeout) {
    first_call = true;
}

void SecondState::execute(ManagerStrategy *manager){
    //implement here
}   

int SecondState::getStateName(){
    return STATE_2;
}

/*---------------------------- ThirdState ---------------------------*/
ThirdState::ThirdState() : State() {
    first_call = true;
}

ThirdState::ThirdState(double timeout) : State(timeout) {
    first_call = true;
} 

void ThirdState::execute(ManagerStrategy *manager){
    //implement here
}

int ThirdState::getStateName(){
    return STATE_3;
}

/*---------------------------- FourthState ---------------------------*/
FourthState::FourthState() : State() {
    first_call = true;
}

FourthState::FourthState(double timeout) : State(timeout) {
    first_call = true;
}

void FourthState::execute(ManagerStrategy *manager){
    //implement here
}

int FourthState::getStateName(){
    return STATE_4;
}

/*---------------------------- EndState ---------------------------*/
EndState::EndState() : State() {
    first_call = true;
}

EndState::EndState(double timeout) : State(timeout) {
    first_call = true;
} 

void EndState::execute(ManagerStrategy *manager){
    //implement here
}

int EndState::getStateName(){
    return END_STATE;
}