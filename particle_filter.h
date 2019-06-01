#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"

#include <string>
#include <random>
#include <numeric>
#include <map>

using namespace std;

struct Particle {
    int id;
    double x;
    double y;
    double theta;
    double weight;
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
};

class ParticleFilter{
    private:
        int num_particles_;
        double std_pos_[3];
        double std_landmark_[2]; // we can remove this 
        double gauss_norm_;
        double norm_x_;
        double norm_y_; 
        vector<double> weights_;
        vector<Particle> particles_;
        vector<Particle> new_particles_;
        vector<Map::single_landmark_s> landmarks_;
        double sensor_range_;

        // add sampling engine to avoid reinitalizing everytime


        bool is_initialized_;

    public:
        ParticleFilter();
        ParticleFilter(double x, double y, double theta, double std[], 
                       double std_landmark[], int n_particles, const Map &map_landmarks, double sensor_range);
        // ~ParticleFilter();   TODO: delete pointers
        void init(double x, double y, double theta, double std[], 
                  double std_landmark[], int n_particles, const Map &map_landmarks, double sensor_range);
        void prediction(double delta_t, double velocity, double yaw_rate);  // need not use std_pos here...
        void dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations);
        void updateWeights(const vector<LandmarkObs> &observations);
        void resample();
        void SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y);

        string getAssociations(Particle best);
        string getSenseCoord(Particle best, string coord);

        const bool initialized() const { return is_initialized_;}
        vector<Particle> getParticles(); // needed for grading...
        //void resetDistributions(); // reset normal distributions
};
#endif

