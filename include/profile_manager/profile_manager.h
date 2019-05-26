#ifndef PROFILE_MANAGER_H
#define PROFILE_MANAGER_H

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <profile_manager/config.h>

class ProfileManager{
    protected:
        bool starting;
        bool isReady;

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