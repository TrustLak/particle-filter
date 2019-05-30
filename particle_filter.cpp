#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <map>
#include <string>
#include <iterator>

#include "particle_filter.h"
using namespace std;


ParticleFilter::ParticleFilter(){
    num_particles_ = 0;
    is_initialized_ = false;

}

ParticleFilter::ParticleFilter(double x, double y, double theta, double std[], double std_landmark[], int n_particles){
    init(x, y, theta, std, std_landmark, n_particles);
}

void ParticleFilter::init(double x, double y, double theta, double std[], double std_landmark[], int n_particles){

    num_particles_ = n_particles;
    weights_.resize(num_particles_); 
    particles_.resize(num_particles_);
    new_particles_.resize(num_particles_);
    
    std_[0] = std[0];
    std_[1] = std[1];
    std_[2] = std[2];

    std_landmark_[0] = std_landmark[0]; 
    std_landmark_[1] = std_landmark[1];

    // TODO: fix random seed
    cout << "Fix random seed" << endl;

    for (int i = 0; i < num_particles_; i++){
        // initialize particles from normal distributions
        //double temp = x_distribution_(rand_engine_);
        particles_[i].id = i;
        particles_[i].x = x + x_noise_(engine_);
        particles_[i].y = y + y_noise_(engine_);
        particles_[i].theta = theta + theta_noise_(engine_);
        particles_[i].weight = 1.0;
        // initialize new_particles_ which is a placeholder for resampling.
        new_particles_[i] = particles_[i];
    }

    is_initialized_ = true;

}

void ParticleFilter::predict(double delta_t, double velocity, double yaw_rate){
    // loop over each particle and update the position based on vel and yaw_rate
    for (int i = 0; i < num_particles_; i++){
        if (yaw_rate == 0){
            particles_[i].x += x_noise_(engine_) + velocity * delta_t * cos(particles_[i].theta);
            particles_[i].y += y_noise_(engine_) + velocity * delta_t * sin(particles_[i].theta);
            particles_[i].theta += theta_noise_(engine_);
        }
        else {
            particles_[i].x += x_noise_(engine_) + 
                (velocity/yaw_rate) * (sin(particles_[i].theta + (yaw_rate * delta_t)) - sin(particles_[i].theta));
            particles_[i].y += y_noise_(engine_) + 
                (velocity/yaw_rate) * (cos(particles_[i].theta) - cos(particles_[i].theta + (yaw_rate * delta_t)));
            particles_[i].theta += theta_noise_(engine_) + yaw_rate * delta_t;
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                            const vector<LandmarkObs> &observations, const Map &map_landmarks){

    return;    
    
}


void ParticleFilter::resample(){
    
    discrete_distribution<int> idx(weights_.begin(), weights_.end());
    for (int i = 0; i < num_particles_; i++){
        new_particles_[i] = particles_[idx(engine_)];
    }

    particles_ = new_particles_; // no need to worry about shallow copy
}


Particle ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
    vector<double> v;

    if (coord == "X") {
        v = best.sense_x;
    } else {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}