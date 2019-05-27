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

int Profiles::getNearest(){
    if(profiles.size() == 0)
        return -1;

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

// return x^2 + y^2 not sqrt(x^2 + y^2)
double Profiles::distanceOf(const Profile &p){
    return pow(p.focus_point.x, 2) + pow(p.focus_point.y, 2);
}

bool Profiles::wasAll(State &state){
    int current_state = state.getStateName();
    for(auto profile : profiles){
        if(!profile.wasActive[current_state]){
            return false;
        }
    }
    return true;
}