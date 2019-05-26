#include <profile_manager/profile.h>

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