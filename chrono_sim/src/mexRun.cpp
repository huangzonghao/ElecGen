#include "mex.h"
#include <Eigen/Core>
#include <iostream>

int launch_simulation(std::string urdf_file,
                      Eigen::MatrixXd heightmap,
                      Eigen::MatrixXd waypoints);

void funtion_usage(){
    std::cout << std::endl
              << "mexFunction Usage:" << std::endl
              <<  "    mexFunction(urdf_filename, heightmap, waypoints)"
              << std::endl;
}

// Param: robot_file - string
//        heightmap  - matrix
//        waypoints  - 3xN matrix (matlab is column major)
//        TODO: motor information
void mexFunction (int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {

    // quick sanity check on input arguments
    bool print_usage = false;
    std::cout << std::endl;
    if (nrhs > 3){
        std::cout << "Too many input arguments" << std::endl;
        print_usage = true;
    }
    if (mxGetClassID(prhs[0]) != mxCHAR_CLASS){
        std::cout << "First argument should be urdf file name" << std::endl;
        print_usage = true;
    }
    if (mxGetClassID(prhs[1]) != mxDOUBLE_CLASS){
        std::cout << "Heightmap should be of double values" << std::endl;
        print_usage = true;
    }
    if (mxGetNumberOfDimensions(prhs[1]) != 2){
        // color map is passed
        std::cout << "Heightmap should be a 2D matrix" << std::endl;
        print_usage = true;
    }
    if (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS){
        std::cout << "Waypoints should be of double values" << std::endl;
        print_usage = true;
    }
    if (mxGetNumberOfDimensions(prhs[2]) != 2 || *(mxGetDimensions(prhs[2])) != 3){
        // wrong format
        std::cout << "Waypoints should be organized to 3xN matrix" << std::endl;
        print_usage = true;
    }

    if (print_usage){
        funtion_usage();
        std::cout << "Error occurred, mex function returned" << std::endl;
        return;
    }

    // Extract data out of Matlab
    char tmp_char_array[256];
    mxGetString(prhs[0], tmp_char_array, 256);
    std::string urdf_file(tmp_char_array);

    // Note both matlab matrix and eigen matrix are column major
    const mwSize *heightmap_dims = mxGetDimensions(prhs[1]);
    const mwSize *waypoint_dims = mxGetDimensions(prhs[2]);
    // Need to make Map const, otherwise a change in map will result the change in Matlab
    const Eigen::MatrixXd heightmap = Eigen::Map<Eigen::MatrixXd>(mxGetPr(prhs[1]), heightmap_dims[0], heightmap_dims[1]);
    const Eigen::MatrixXd waypoints = Eigen::Map<Eigen::MatrixXd>(mxGetPr(prhs[2]), waypoint_dims[0], waypoint_dims[1]);

    // std::cout << urdf_file << std::endl;
    // std::cout <<  heightmap << std::endl << " ---------------" << std::endl << waypoints << std::endl;

    launch_simulation(urdf_file, heightmap, waypoints);
    return;
}
