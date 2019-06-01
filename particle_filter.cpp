
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

#include "particle_filter.h"
using namespace std;


ParticleFilter::ParticleFilter(){
    num_particles_ = 0;
    is_initialized_ = false;

}

ParticleFilter::ParticleFilter(double x, double y, double theta, double std[], 
                       double std_landmark[], int n_particles, const Map &map_landmarks, double sensor_range){
    init(x, y, theta, std, std_landmark, n_particles, map_landmarks, sensor_range_);
}

void ParticleFilter::init(double x, double y, double theta, double std[], 
                       double std_landmark[], int n_particles, const Map &map_landmarks, double sensor_range){

    num_particles_ = n_particles;
    weights_.resize(num_particles_); 
    particles_.resize(num_particles_);
    new_particles_.resize(num_particles_);

    std_pos_[0] = std[0];
    std_pos_[1] = std[1];
    std_pos_[2] = std[2];

    std_landmark_[0] = std_landmark[0]; 
    std_landmark_[1] = std_landmark[1];

    /*I
    normal_distribution<double> x_new(0, std_pos_[0]);
    normal_distribution<double> y_new(0, std_pos_[1]);
    normal_distribution<double> theta_new(0, std_pos_[2]);

    x_noise_ = x_new;
    y_noise_ = y_new;
    theta_noise_ = theta_new;
    // fix random seed
    //engine_.seed(42);
    */
    std::random_device rd{};
    std::mt19937 engine_{rd()};
    normal_distribution<double> x_noise_(0, std_pos_[0]);
    normal_distribution<double> y_noise_(0, std_pos_[1]);
    normal_distribution<double> theta_noise_(0, std_pos_[2]);


    gauss_norm_ = 1 / (2 * M_PI * std_landmark_[0] * std_landmark_[1]);  // can remove std_landmark from here
    norm_x_ = 2 * std_landmark_[0] * std_landmark_[0];
    norm_y_ = 2 * std_landmark_[1] * std_landmark_[1];

    landmarks_ = map_landmarks.landmark_list;    
    sensor_range_ = sensor_range;
    //default_random_engine engine_;

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
    //resetDistributions();
}

void ParticleFilter::prediction(double delta_t, double velocity, double yaw_rate){
    // loop over each particle and update the position based on vel and yaw_rate
    //random_device rand_dev_;
    //default_random_engine engine_;

    std::random_device rd{};
    std::mt19937 engine_{rd()};
    normal_distribution<double> x_noise_(0, std_pos_[0]);
    normal_distribution<double> y_noise_(0, std_pos_[1]);
    normal_distribution<double> theta_noise_(0, std_pos_[2]);

    for (int i = 0; i < num_particles_; i++){
        if (fabs(yaw_rate) == 0){
            particles_[i].x += x_noise_(engine_) + velocity * delta_t * cos(particles_[i].theta);
            particles_[i].y += y_noise_(engine_) + velocity * delta_t * sin(particles_[i].theta);
            particles_[i].theta += theta_noise_(engine_); //+ yaw_rate * delta_t;
        }
        else {
            particles_[i].x += x_noise_(engine_) + 
                (velocity/yaw_rate) * (sin(particles_[i].theta + (yaw_rate * delta_t)) - sin(particles_[i].theta));
            particles_[i].y += y_noise_(engine_) + 
                (velocity/yaw_rate) * (cos(particles_[i].theta) - cos(particles_[i].theta + (yaw_rate * delta_t)));
            particles_[i].theta += theta_noise_(engine_) + yaw_rate * delta_t;
        }

    }
    //resetDistributions();
}

void ParticleFilter::updateWeights(const vector<LandmarkObs> &observations){

    for (int i = 0; i < num_particles_; i++){
        double pi_gauss = 1.0;  // 

        for (unsigned int j = 0; j < observations.size(); j++){
            // vehicle to map coordinates
            double x_map = observations[j].x * cos(particles_[i].theta) - observations[j].y * sin(particles_[i].theta) + particles_[i].x;
            double y_map = observations[j].y * sin(particles_[i].theta) + observations[j].y * cos(particles_[i].theta) + particles_[i].y;

            // landmarks within sensor range
            vector<double> delta_distances; // landmark-observation mean values; assume very large value initially  
            for (unsigned int k = 0; k < landmarks_.size(); k++){
                // square euclidean distance 
                double particle_dist = pow(particles_[i].x - landmarks_[k].x_f, 2) + pow(particles_[i].y - landmarks_[k].y_f, 2);
                if (particle_dist <= sensor_range_*sensor_range_){
                    delta_distances.push_back( pow(x_map - landmarks_[k].x_f, 2) + pow(y_map - landmarks_[k].y_f, 2) );
                }
                else{
                    delta_distances.push_back(9999999.0);
                }
            }
            // associate with nearest landmark 
            int min_index = distance(delta_distances.begin(), min_element(delta_distances.begin(),delta_distances.end()));

            // update the guassian (product of guassians)
            double exponent  = pow(x_map - landmarks_[min_index].x_f, 2) / norm_x_ 
                             + pow(y_map - landmarks_[min_index].y_f, 2) / norm_y_;
            pi_gauss *= gauss_norm_ * exp(-exponent);
        }
        particles_[i].weight = pi_gauss;
        weights_[i] = particles_[i].weight;

    }
}


void ParticleFilter::resample(){
    
        std::random_device rd{};
        std::mt19937 engine_{rd()};


    discrete_distribution<int> idx(weights_.begin(), weights_.end());
    for (int i = 0; i < num_particles_; i++){
        new_particles_[i] = particles_[idx(engine_)];
    }
    particles_ = new_particles_; // no need to worry about shallow copy
}


void ParticleFilter::SetAssociations(Particle& particle, 
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


vector<Particle> ParticleFilter::getParticles(){
    return particles_;
}

/*
void ParticleFilter::resetDistributions(){
    x_noise_.reset();
    y_noise_.reset();
    theta_noise_.reset();
}
*/


