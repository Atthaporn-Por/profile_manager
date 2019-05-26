#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>

class State{
    private:
        ros::Time before;
        ros::Time latest;
        bool first_call;
    public:
        State();
        virtual void execute();
        virtual std::string getStateName();
};

class IdleState : public State{
    public:
        IdleState();
        void execute();
        std::string getStateName();
};

class FirstState : public State{
    public:
        FirstState();
        void execute();
        std::string getStateName();
};

class SecondState : public State{
    public:
        SecondState();
        void execute();
        std::string getStateName();
};

class ThirdState : public State{
    public:
        ThirdState();
        void execute();
        std::string getStateName();
};

class FourthState : public State{
    public:
        FourthState();
        void execute();
        std::string getStateName();
};

class EndState : public State{
    public:
        EndState();
        void execute();
        std::string getStateName();
};




#endif