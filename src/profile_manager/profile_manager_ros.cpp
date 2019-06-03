#include <profile_manager/profile_manager_ros.h>

ProfileManagerRos::ProfileManagerRos(ros::NodeHandle &n) : ProfileManager(n){
    //implement here
    this->n = n;
    this->action_pub = n.advertise<dinsow_msgs::DinsowAction>("/dinsow_action", 1);
    this->state_pub = n.advertise<std_msgs::String>(config.resolveTopic("state"), 1, true);
    this->focus_point_pub = n.advertise<geometry_msgs::Point>(config.resolveTopic("focus_point"), 1);
    this->people_sub = n.subscribe<skeleton_msgs::People>("/body/dinsow4/skeleton/people", 5,
        boost::bind(&ProfileManagerRos::peopleCb, this, _1)
    );
    this->db_change_sub = n.subscribe<dinsow_msgs::DBChange>("/db_change", 1,
        boost::bind(&ProfileManagerRos::dbChangeCb, this, _1)
    );
    this->start_service = n.advertiseService(config.resolveTopic("start"),
        &ProfileManagerRos::startService, this
    );
    this->stop_service = n.advertiseService(config.resolveTopic("stop"),
        &ProfileManagerRos::stopService, this
    );
    this->dinsow_config_client = n.serviceClient<dinsow_msgs::DinsowStore>("dinsow_config");
    this->worker = boost::thread(boost::bind(&ProfileManagerRos::run, this));
}

void ProfileManagerRos::peopleCb(const skeleton_msgs::People::ConstPtr &p){
    ROS_DEBUG("recived data from /body/dinsow4/skeleton/people size : %d", (int)p->people.size());
    if(p->people.size() > 0){
        if(starting)
            isReady = true;
    }else{
        isReady = false;
    }
    for(auto person : p->people){
        profiles.add(state, std::atoi(person.name.c_str()), person.skeleton.joints[0].position);
    }
}   

void  ProfileManagerRos::dbChangeCb(const dinsow_msgs::DBChange::ConstPtr &p){
    ROS_DEBUG("recived data from /db_change");
    //implement here
}

void ProfileManagerRos::publish(const dinsow_msgs::DinsowAction &action){
    action_pub.publish(action);
}

void ProfileManagerRos::publish(const std_msgs::String &s){
    state_pub.publish(s);
}

void ProfileManagerRos::publish(const std::string &s){
    if(s == current_state.data)
        return;
    current_state.data = s;
    publish(current_state);
}

void ProfileManagerRos::publish(const geometry_msgs::Point &p){
    focus_point_pub.publish(p);
}

bool ProfileManagerRos::startService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("%s is started", ros::this_node::getName().c_str());
    return start();
}

bool ProfileManagerRos::stopService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    ROS_INFO("%s is started", ros::this_node::getName().c_str());
    return stop();
}

void ProfileManagerRos::run(){
    //implement here
    ros::Rate r(config.getFrequenzy());
    boost::mutex::scoped_lock lock(mutex);
    while(ros::ok()){
        if(isReady){
            ROS_INFO_ONCE("ready");
            while(!starting){
                cv.wait(lock, [this]{return starting;});
            }
            if(manager_strategy->doAction()){
                ROS_INFO("do_action");
                publish(manager_strategy->getAction());
                manager_strategy->setDoAction(false);
            }
            geometry_msgs::Point point= manager_strategy->getFocusPoint();
            if(manager_strategy->getCurrentStateName() == "FOLLOWING_STATE"){
                publish(point);
            }
        }
        publish(manager_strategy->getCurrentStateName());
        profiles.update();
        r.sleep();
        ros::spinOnce();
    }
}

void ProfileManagerRos::callDinsowConfig(){
    //implement here
}