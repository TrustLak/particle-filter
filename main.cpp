#include <iostream>
#include "particle_filter.h"
using namespace std;

int main(){
    ParticleFilter filter;
    double x = 1.0;
    double y = 1.9;
    double theta = 0.0;
    double *std_pos = new double[3];
    double std_landmark[2] = {0.0 , 0.0};

    std_pos[0] = 1.0;
    std_pos[1] = 1.0;
    std_pos[2] = 1.0;


    // check if init and non-default constructors work
    filter.init(x, y, theta, std_pos, std_landmark, 200);
    ParticleFilter f(x,y,theta, std_pos, std_landmark, 100);


    return 0;
}