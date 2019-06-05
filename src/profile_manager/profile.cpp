#include <profile_manager/profile.h>
#include <exception>

Profiles::Profiles(){
    this->profiles = std::vector<Profile>();
}


void Profiles::add(State *state, int id, const geometry_msgs::Point &p){
    if(p.z > 0.6 || p.x > 3.5 || (abs(p.x) + abs(p.y) + abs(p.z) < 0.00000002 )){
        return;
    }
    for(auto profile = profiles.begin(); profile != profiles.end(); profile++){
        if(profile->id == id){
            ROS_WARN("Fouund id : %d", id);
            profile->focus_point = p;
            profile->seen_time = ros::Time::now();
            setWasActive(state, id);
            return;
        } 
    }
    ROS_WARN("Not Fouund id : %d", id);
    profiles.push_back(Profile(id, p));
    setWasActive(state, id);
}

void Profiles::erase(int id){
    for(auto profile = profiles.begin(); profile != profiles.end(); profile++){
        if(profile->id == id){
            profiles.erase(profile);
        }
    }
}

void Profiles::update(){
    for(auto profile = profiles.begin(); profile != profiles.end(); profile++){
        if(ros::Time::now() - profile->seen_time > ros::Duration(1.2)){
            profile = profiles.erase(profile);
            if(profile == profiles.end())
                break;
        }
    }
    // for(auto profile = profiles.begin(); profile != profiles.end(); profile++){
    //     ROS_INFO("id : %d , {x: %f, y :%f, z: %f}", profile->id, profile->focus_point.x, profile->focus_point.y, profile->focus_point.z);
    // }
    // ROS_INFO("============================================================");
}

void Profiles::setWasActive(State *state, int id){
    for(auto profile = profiles.begin(); profile != profiles.end();profile++){
        if(profile->id == id){
            setWasActive(state, profile);
            break;
        }
    }
}

void Profiles::setWasActive(State *state, std::vector<Profile>::iterator i){
    for(int s = state->getState(); s>=STATE_1; s--){
        if(s == FOLLOWING_STATE){
            break;
        }
        i->wasActive[s] = true;
    }
}

void Profiles::setWasActiveAll(State *state){
    for(auto profile = profiles.begin(); profile != profiles.end(); profile++){
        setWasActive(state, profile);
    }
}

// return x^2 + y^2 not sqrt(x^2 + y^2)
double Profiles::distanceOf(const Profile &p){
    return pow(p.focus_point.x, 2) + pow(p.focus_point.y, 2);
}

// return x^2 + y^2 not sqrt(x^2 + y^2)
double Profiles::distanceOf(const geometry_msgs::Point &p){
    return pow(p.x, 2) + pow(p.y, 2);
}

double Profiles::getNearestDistance(){
    return distanceOf(getNearestPoint());
}

bool Profiles::wasAll(State &state){
    int current_state = state.getState();
    return wasAll(current_state);
}

bool Profiles::wasAll(int state){
    for(auto profile : profiles){
        if(!profile.wasActive[state]){
            return false;
        }
    }
    return true;
}

int Profiles::getNearestId(){
    if(isEmpty()){
        ROS_ERROR("profiles is empty");
        throw "profiles is empty";
    }

    auto profile = profiles.begin();
    auto nearest_person = profiles.begin();
    profile++;
    for(profile; profile != profiles.end(); profile++){
        if(distanceOf(*profile) < distanceOf(*nearest_person)){
            nearest_person = profile;
        }
    }
    return nearest_person->id;
}

geometry_msgs::Point Profiles::getNearestPoint(){
    return getFocusPoint(getNearestId());
}

geometry_msgs::Point Profiles::getFocusPoint(int id){
     for(auto profile : profiles){
         if(profile.id == id){
             return profile.focus_point;
         }
     }
     return getNearestPoint();
     ROS_ERROR("no id : %d in profiles", id);
     throw "no id : %d in profiles", id;
}

geometry_msgs::Point Profiles::getNearestPoint(State& state, double max_distance){
    if(isEmpty()){
        ROS_ERROR("profiles is empty");
        throw "profiles is empty";
    }
    if(wasAll(state)){
        return getNearestPoint();
    }else{
        std::vector<Profile>::iterator p = profiles.end();
        for(auto q = profiles.begin(); q != profiles.end();q++){
            if(!q->wasActive[state.getState()]){
                if(distanceOf(*q) < pow(max_distance, 2)){
                    if(p == profiles.end()){
                        p = q;
                    }else if(distanceOf(*q) < distanceOf(*p) ){
                        p = q;
                    }
                }
            }
        }
        if(p == profiles.end()){
            return getNearestPoint();
        }else{
            return p->focus_point;
        }
    }
}

bool Profiles::isEmpty(){
    return profiles.size() == 0;
}