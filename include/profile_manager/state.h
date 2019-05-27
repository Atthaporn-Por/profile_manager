#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>

class State{
    private:
        ros::Time before;
        ros::Time latest;
        ros::Time timeout;
        bool first_call;
    public:
        State();
        State(double timeout);
        virtual void execute();
        virtual std::string getStateName();
};

class IdleState : public State{
    public:
        IdleState();
        IdleState(double timeout);
        void execute();
        std::string getStateName();
};

class FirstState : public State{
    public:
        FirstState();
        FirstState(double timeout);
        void execute();
        std::string getStateName();
};

class SecondState : public State{
    public:
        SecondState();
        SecondState(double timeout);
        void execute();
        std::string getStateName();
};

class ThirdState : public State{
    public:
        ThirdState();
        ThirdState(double timeout);
        void execute();
        std::string getStateName();
};

class FourthState : public State{
    public:
        FourthState();
        FourthState(double timeout);
        void execute();
        std::string getStateName();
};

class EndState : public State{
    public:
        EndState();
        EndState(double timeout);
        void execute();
        std::string getStateName();
};




#endif