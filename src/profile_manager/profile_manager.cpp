#include <profile_manager/profile_manager.h>

ProfileManager::ProfileManager(){
    starting = false;
    isReady = false;
    this->config = Config();
    this->profiles = Profiles();
    this->state = new IdleState(&profiles, config.getDistanceArray(), config.getTimeoutArray());
    this->manager_strategy = new ManagerStrategy(*state, config);
}

ProfileManager::ProfileManager(ros::NodeHandle &n){
    boost::mutex::scoped_lock lock(mutex);
    starting = true;
    isReady = false;
    
    this->config = Config(n);
    this->profiles = Profiles();
    this->state = new IdleState(&profiles, config.getDistanceArray(), config.getTimeoutArray());
    this->manager_strategy = new ManagerStrategy(*state, config);
}

bool ProfileManager::start(){
    starting = true;
    cv.notify_all();
    return true;
}

bool ProfileManager::stop(){
    starting = false;
    return true;
}