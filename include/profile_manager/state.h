#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>
#include <profile_manager/constant.h>
#include <profile_manager/manager_strategy.h>

class State{
    protected:
        ros::Time before;
        ros::Time latest;
        ros::Duration timeout;
        bool first_call;
    public:
        State();
        State(double timeout);
        virtual void execute(ManagerStrategy *manager);
        virtual int getStateName();
};

class IdleState : public State{
    public:
        IdleState();
        IdleState(double timeout);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

class FirstState : public State{
    public:
        FirstState();
        FirstState(double timeout);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

class SecondState : public State{
    public:
        SecondState();
        SecondState(double timeout);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

class ThirdState : public State{
    public:
        ThirdState();
        ThirdState(double timeout);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

class FourthState : public State{
    public:
        FourthState();
        FourthState(double timeout);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

class EndState : public State{
    public:
        EndState();
        EndState(double timeout);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

#endif