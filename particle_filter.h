#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_function.h"

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
        double std_[3];
        double std_landmark_[2]; 
        vector<double> weights_;
        vector<Particle> particles_;
        vector<Particle> new_particles_;

        // add sampling engine to avoid reinitalizing everytime
        random_device rand_dev_{};
        default_random_engine engine_{rand_dev_()};
        normal_distribution<double> x_noise_;
        normal_distribution<double> y_noise_;
        normal_distribution<double> theta_noise_;

        bool is_initialized_;

    public:
        ParticleFilter();
        ParticleFilter(double x, double y, double theta, double std[], double std_landmark[], int n_particles);
        // ~ParticleFilter();   TODO: delete pointers
        void init(double x, double y, double theta, double std[], double std_landmark[], int n_particles);
        void predict(double delta_t, double velocity, double yaw_rate);  // need not use std_pos here...
        void dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations);
        void updateWeights(double sensor_range, double std_landmark[], 
                            const vector<LandmarkObs> &observations, const Map &map_landmarks);
        void resample();
        Particle SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y);

        string getAssociations(Particle best);
        string getSenseCoord(Particle best, string coord);
        string getSenseX(Particle best);
        string getSenseY(Particle best);
        const bool initialized() const { return is_initialized_;}
};
#endif