#include <mex.h>
#include <Eigen/Core>
#include <iostream>
#include <memory>

// TODO: Ideally only need to pass env_file, but currently having difficulties
// reading in bitmap in c++
void launch_simulation(const std::string& urdf_file,
                       const std::string& env_file,
                       const std::shared_ptr<const Eigen::MatrixXd>& heightmap,
                       const std::shared_ptr<const Eigen::MatrixXd>& waypoints);

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
    if (nrhs > 4){
        std::cout << "Too many input arguments" << std::endl;
        print_usage = true;
    }
    if (mxGetClassID(prhs[0]) != mxCHAR_CLASS){
        std::cout << "First argument should be urdf file name" << std::endl;
        print_usage = true;
    }
    if (mxGetClassID(prhs[1]) != mxCHAR_CLASS){
        std::cout << "First argument should be urdf file name" << std::endl;
        print_usage = true;
    }
    if (mxGetClassID(prhs[2]) != mxDOUBLE_CLASS){
        std::cout << "Heightmap should be of double values" << std::endl;
        print_usage = true;
    }
    if (mxGetNumberOfDimensions(prhs[2]) != 2){
        // color map is passed
        std::cout << "Heightmap should be a 2D matrix" << std::endl;
        print_usage = true;
    }
    if (mxGetClassID(prhs[3]) != mxDOUBLE_CLASS){
        std::cout << "Waypoints should be of double values" << std::endl;
        print_usage = true;
    }
    if (mxGetNumberOfDimensions(prhs[3]) != 2 || *(mxGetDimensions(prhs[3])) != 3){
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
    mxGetString(prhs[1], tmp_char_array, 256);
    std::string env_file(tmp_char_array);

    // Note both matlab matrix and eigen matrix are column major
    const mwSize *heightmap_dims = mxGetDimensions(prhs[2]);
    const mwSize *waypoint_dims = mxGetDimensions(prhs[3]);
    // Need to make Map const, otherwise a change in map will result the change in Matlab
    std::shared_ptr<const Eigen::MatrixXd> heightmap = std::make_shared<const Eigen::MatrixXd>(Eigen::Map<Eigen::MatrixXd>(mxGetPr(prhs[2]), heightmap_dims[0], heightmap_dims[1]));
    std::shared_ptr<const Eigen::MatrixXd> waypoints = std::make_shared<const Eigen::MatrixXd>(Eigen::Map<Eigen::MatrixXd>(mxGetPr(prhs[3]), waypoint_dims[0], waypoint_dims[1]));

    launch_simulation(urdf_file, env_file, heightmap, waypoints);
    return;
}
