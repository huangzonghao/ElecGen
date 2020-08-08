#include "mex.h"
#include <iostream>

int launch_simulation(double init_x=0.0, double init_y=0.0,
                      double goal_x=0.0, double goal_y=0.0,
                      std::string urdf_filename="",
                      std::string env_filename="");


void mexFunction (int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {

    mxDouble* init_x = mxGetDoubles(prhs[0]);
    mxDouble* init_y = mxGetDoubles(prhs[1]);
    mxDouble* goal_x = mxGetDoubles(prhs[2]);
    mxDouble* goal_y = mxGetDoubles(prhs[3]);
    char env_filename[256];
    char urdf_filename[256];
    mxGetString(prhs[4], urdf_filename, 256);
    mxGetString(prhs[5], env_filename, 256);
    std::string uf_string(urdf_filename);
    std::string ef_string(env_filename);

    launch_simulation(*init_x, *init_y, *goal_x, *goal_y, urdf_filename, env_filename);
    return;
}
