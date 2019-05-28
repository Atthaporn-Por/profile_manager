#include <profile_manager/profile.h>
#include <exception>
Profiles::Profiles(){
    this->profiles = std::vector<Profile>();
}


void Profiles::add(int id, const geometry_msgs::Point &p){
    for(auto profile = profiles.begin(); profile != profiles.end(); profile++){
        if(profile->id == id){
            profile->focus_point = p;
            break;
        }
    }
    profiles.push_back(Profile(id, p));
}

void Profiles::erase(int id){
    for(auto profile = profiles.begin(); profile != profiles.end(); profile++){
        if(profile->id == id){
            profiles.erase(profile);
        }
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
    for(auto profile : profiles){
        if(!profile.wasActive[current_state]){
            return false;
        }
    }
    return true;
}

int Profiles::getNearestId(){
    if(isEmpty())
        throw "profiles is empty";

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
     throw "no id : %d in profiles", id;
}

bool Profiles::isEmpty(){
    return profiles.size() == 0;
}