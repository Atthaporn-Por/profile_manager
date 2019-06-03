#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>
#include <profile_manager/constant.h>
#include <profile_manager/manager_strategy.h>
#include <profile_manager/profile.h>

//send focus_point by setter of manager_strategy


class ManagerStrategy;
class Profiles;

class State{
    protected:
        ros::Time before;
        ros::Time latest;
        double *timeout;
        double *distance;
        bool first_call;
        Profiles* profiles;
        int important_id;
    public:
        State();
        State(Profiles* profiles, double *distance, double *timeout);
        void changeState(ManagerStrategy *manager, int state);
        virtual void execute(ManagerStrategy *manager) = 0;
        virtual int getState() = 0;
        virtual std::string getStateName() = 0;
};

class IdleState : public State{
    public:
        IdleState(Profiles* profiles, double *distance, double *timeout);
        void execute(ManagerStrategy *manager);
        int getState();
        std::string getStateName();
};

class FirstState : public State{
    public:
        FirstState(Profiles* profiles, double *distance, double *timeout);
        void execute(ManagerStrategy *manager);
        int getState();
        std::string getStateName();
};

class SecondState : public State{
    public:
        SecondState(Profiles* profiles, double *distance, double *timeout);
        void execute(ManagerStrategy *manager);
        int getState();
        std::string getStateName();
};

class ThirdState : public State{
    public:
        ThirdState(Profiles* profiles, double *distance, double *timeout);
        void execute(ManagerStrategy *manager);
        int getState();
        std::string getStateName();
};

class FollowingState : public State{
    public:
        FollowingState(Profiles* profiles, double *distance, double *timeout);
        void execute(ManagerStrategy *manager);
        int getState();
        std::string getStateName();
};

class EndState : public State{
    public:
        EndState(Profiles* profiles, double *distance, double *timeout);
        void execute(ManagerStrategy *manager);
        int getState();
        std::string getStateName();
};

#endif