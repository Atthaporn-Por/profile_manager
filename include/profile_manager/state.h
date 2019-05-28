#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>
#include <profile_manager/constant.h>
#include <profile_manager/manager_strategy.h>
#include <profile_manager/profile.h>

//send focus_point by setter of manager_strategy


class ManagerStrategy;

class State{
    protected:
        ros::Time before;
        ros::Time latest;
        ros::Duration timeout;
        double *distance;
        bool first_call;
        Profiles* profiles;
    public:
        State();
        State(Profiles* profiles, double *distance);
        void setTimeout(double timeout);
        void setDistance(double distance);
        virtual void execute(ManagerStrategy *manager) = 0;
        virtual int getStateName() = 0;
};

class IdleState : public State{
    public:
        IdleState(Profiles* profiles, double *distance);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

class FirstState : public State{
    public:
        FirstState(Profiles* profiles, double *distance);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

class SecondState : public State{
    public:
        SecondState(Profiles* profiles, double *distance);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

class ThirdState : public State{
    public:
        ThirdState(Profiles* profiles, double *distance);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

class FourthState : public State{
    public:
        FourthState(Profiles* profiles, double *distance);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

class EndState : public State{
    public:
        EndState(Profiles* profiles, double *distance);
        void execute(ManagerStrategy *manager);
        int getStateName();
};

#endif