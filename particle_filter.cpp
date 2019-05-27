# include "particle_filter.h"
#include <iostream>
using namespace std;


ParticleFilter::ParticleFilter(){
    num_particles_ = 0;
    is_initialized_ = false;

}

ParticleFilter::ParticleFilter(double x, double y, double theta, double std[], int n_particles){
    init(x, y, theta, std);
};

void ParticleFilter::init(double x, double y, double theta, double std[], int n_particles){

    num_particles_ = 200;
    weights_.resize(num_particles_); 
    particles_.resize(num_particles_); 
    std_ = std;

    is_initialized_ = true;

    cout << "Successfully executed init(..)" << endl;
};
