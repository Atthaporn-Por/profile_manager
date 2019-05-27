#ifndef PROFILE_MANAGER_H
#define PROFILE_MANAGER_H

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <profile_manager/config.h>
#include <profile_manager/state.h>
#include <profile_manager/manager_strategy.h>
#include <profile_manager/profile.h>

class ProfileManager{
    protected:
        bool starting;
        bool isReady;

        State *state;
        Config config;
        Profiles profiles;

        ManagerStrategy *manager_strategy;
        
        boost::thread worker;
        boost::mutex mutex;
        boost::condition_variable cv;

    public:
        ProfileManager();
        ProfileManager(ros::NodeHandle &n);

    public:
        virtual void run() = 0;
        bool start();
        bool stop();
    
};

#endif