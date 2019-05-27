#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_function.h"
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
        bool is_initialized_;
        double *std_;
        vector<double> weights_;
        vector<Particle> particles_;

    public:
        ParticleFilter();
        ParticleFilter(double x, double y, double theta, double std[], int n_particles);
        // ~ParticleFilter();   TODO
        void init(double x, double y, double theta, double std[], int n_particles);
        void predict(double delta_t, double std_pos[], double velocity, double yaw_rate);  // need not use std_pos here...
        void dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations);
        void updateWeights(double sensor_range, double std_landmark[], const vector<LandmarkObs> &observations, const Map &map_landmarks);
        void resample();
        Particle SetAssociations(Particle& particle, const std::vector<int>& associations,
		                     const std::vector<double>& sense_x, const std::vector<double>& sense_y);
        string getAssociations(Particle best);
        string getSenseX(Particle best);
        string getSenseY(Particle best);
        const bool initialized() const { return is_initialized_;}
};
#endif