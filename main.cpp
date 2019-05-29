#include <iostream>
#include "particle_filter.h"
using namespace std;

int main(){
    ParticleFilter filter;
    double x = 1.0;
    double y = 1.9;
    double theta = 0.0;
    double *std = new double[3];
    std[0] = 1.0;
    std[1] = 1.0;
    std[2] = 1.0;

    // check if init and non-default constructors work
    filter.init(x, y, theta, std, 200);
    ParticleFilter f(x,y,theta,std, 100);


    // test predict

    // test dataAssociation

    // test updateWeights

    // test resample

    return 0;
}