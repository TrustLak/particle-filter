# include "particle_filter.h"
#include <iostream>
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <map>

using namespace std;


ParticleFilter::ParticleFilter(){
    num_particles_ = 0;
    is_initialized_ = false;

}

ParticleFilter::ParticleFilter(double x, double y, double theta, double std[], int n_particles){
    init(x, y, theta, std, n_particles);
};

void ParticleFilter::init(double x, double y, double theta, double std[], int n_particles){

    num_particles_ = n_particles;
    weights_.resize(num_particles_); 
    particles_.resize(num_particles_);
    new_particles_.resize(num_particles_);
    std_ = std;

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

};


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
};


void ParticleFilter::resample(){
    
    discrete_distribution<int> idx(weights_.begin(), weights_.end());
    for (int i = 0; i < num_particles_; i++){
        new_particles_[i] = particles_[idx(engine_)];
    }
    particles_ = new_particles_;
};