#include <profile_manager/config.h>

Config::Config(){
    this->frequenzy = 10.0;
    this->distance[STATE_1] = 0.70;
    this->distance[STATE_2] = 1.00;
    this->distance[STATE_3] = 2.00;
    this->distance[FOLLOWING_STATE] = 3.50;
    this->timeout[STATE_1] = 5.0;
    this->timeout[STATE_2] = 5.0;
    this->timeout[STATE_3] = 5.0;
    this->timeout[FOLLOWING_STATE] = 5.0;
    this->timeout[END_STATE] = 5.0;
    this->short_follow_duration = 20.0;
    this->long_follow_duration = 150.0;
    this->name_space = "/";
    this->node_name = "/profile_manager";
    for(int i=0;i<ACTION_SIZE;i++){
        state_action[i].action_id = "1000" + std::to_string(i+1);
        state_action[i].json_data = "";
        state_action[i].requester = this->node_name;
    }
}

Config::Config(ros::NodeHandle &n){
    double time;
    this->name_space = ros::this_node::getNamespace();
    this->node_name = ros::this_node::getName();
    n.param<double>(resolveTopic("frequenzy"), frequenzy, 10.0);
    n.param<double>(resolveTopic("state_1_distance"), distance[STATE_1], 0.70);
    n.param<double>(resolveTopic("state_2_distance"), distance[STATE_2], 1.00);
    n.param<double>(resolveTopic("state_3_distance"), distance[STATE_3], 2.00);
    n.param<double>(resolveTopic("state_4_distance"), distance[FOLLOWING_STATE], 3.50);
    n.param<std::string>(resolveTopic("idle_state_action"), state_action[IDLE_STATE].action_id, "10001");
    n.param<std::string>(resolveTopic("state_1_action"), state_action[STATE_1].action_id, "10002");
    n.param<std::string>(resolveTopic("state_2_action"), state_action[STATE_2].action_id, "10003");
    n.param<std::string>(resolveTopic("state_3_action"), state_action[STATE_3].action_id, "10004");
    n.param<std::string>(resolveTopic("state_4_action"), state_action[FOLLOWING_STATE].action_id, "10005");
    n.param<std::string>(resolveTopic("end_state_action"), state_action[END_STATE].action_id, "10006");
    n.param<double>(resolveTopic("state_1_timeout"), timeout[STATE_1], 5.0);
    n.param<double>(resolveTopic("state_2_timeout"), timeout[STATE_2], 5.0);
    n.param<double>(resolveTopic("state_3_timeout"), timeout[STATE_3], 5.0);
    n.param<double>(resolveTopic("state_4_timeout"), timeout[FOLLOWING_STATE], 5.0);
    n.param<double>(resolveTopic("end_state_timeout"), timeout[END_STATE], 5.0);
    n.param<double>(resolveTopic("short_follow_duration"), short_follow_duration, 20.0);
    n.param<double>(resolveTopic("long_follow_duration"), long_follow_duration, 150.0);
    for(int i=0;i<ACTION_SIZE;i++){
        state_action[i].json_data = "";
        state_action[i].requester = this->node_name;
    }
}

dinsow_msgs::DinsowAction Config::getAction(int state){
    return state_action[state];
}

std::string Config::resolveTopic(const std::string &topic){
    std::string resolved_topic = "";
    if(this->name_space.length() > 1)
        resolved_topic = resolved_topic + this->name_space + this->node_name + "/" + topic;
    else
        resolved_topic = resolved_topic + this->node_name + "/" + topic;
    return resolved_topic;
}

std::string Config::getBaseFrame(){
    return base_frame;
}

double Config::getFrequenzy(){
    return frequenzy;
}

double Config::getTimeout(int state){
    int key[] = {-1, 0, 1, 2, 3, -1};
    if(key[state] == -1)
        throw "this state dont have timeout";
    return timeout[key[state]];
}

double* Config::getDistanceArray(){
    return distance;
}
double* Config::getTimeoutArray(){
    return timeout;
}