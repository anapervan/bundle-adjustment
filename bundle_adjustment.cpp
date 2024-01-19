#include <ceres/ceres.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>


struct Data {
    int num_cameras;
    int num_points;
    int num_observations;
    Eigen::VectorXi camera_inds;
    Eigen::VectorXi point_inds;
    Eigen::MatrixXd points_uv;
    Eigen::VectorXd camera_params;
    Eigen::VectorXd points_xyz;
};

Data read_data(const std::string& dataset_name) {
    /* 
    camera_ind: (num_observations,) 
        indices of cameras (from 0 to num_cameras - 1) involved in each observation.
    point_ind: (num_observations,) 
        indices of points (from 0 to num_points - 1) involved in each observation.
    points_uv: (num_observations, 2) 
        observed 2D coordinates (u, v) in the image frame for each observations.
    camera_params: (num_cameras, 9) 
        initial estimates of all camera parameters:
            [0, 1, 2] a (Rodrigues) rotation vector 
            [3, 4, 5] a translation vector
            [6, 7, 8] a focal distance and two distortion parameters   
    points_xyz: (num_points, 3)
        initial estimates of 3D point coordinates (x, y, z) in the world frame.
    */

    // Read data from file
    std::ifstream file(dataset_name);

    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + dataset_name);    
    }

    Data data;
    file >> data.num_cameras >> data.num_points >> data.num_observations;

    std::cout << "num_cameras: " << data.num_cameras << std::endl;
    std::cout << "num_points: " << data.num_points << std::endl;
    std::cout << "num_observations: " << data.num_observations << std::endl;

    // Resize Eigen structures
    data.camera_inds.resize(data.num_observations);
    data.point_inds.resize(data.num_observations);
    data.points_uv.resize(data.num_observations, 2);
    data.camera_params.resize(data.num_cameras * 9);
    data.points_xyz.resize(data.num_points * 3);

    // Read and assign data
    for (int i = 0; i < data.num_observations; ++i) {
        file >> data.camera_inds[i] >> data.point_inds[i] >> data.points_uv(i,0) >> data.points_uv(i,1);
    }

    for (int i = 0; i < data.num_cameras * 9; ++i) {
        file >> data.camera_params[i];
    }

    for (int i = 0; i < data.num_points * 3; ++i) {
        file >> data.points_xyz[i];
    }

    file.close();
    return data;
}

template <typename T>Eigen::Matrix<T, 3, 1> rotate_point(const Eigen::Matrix<T, 3, 1> point_xyz, const Eigen::Matrix<T, 3, 1> rotation_vec) 
{    //Rotate 3D points by the estimated rotation vectors. """

    T theta = rotation_vec.norm();

    Eigen::Matrix<T, 3, 1> k;
    if (theta != 0.0) {
        k = rotation_vec / theta;
    } else {
        k = Eigen::Matrix<T, 3, 1> (T(0.0), T(0.0), T(0.0));
    }

    T dot = point_xyz.dot(k);
    T cos_theta = ceres::cos(theta);
    T sin_theta = ceres::sin(theta);
    Eigen::Matrix<T, 3, 1> rotated_point = point_xyz * cos_theta +
                                            k.cross(point_xyz) * sin_theta +
                                            k * dot * (T(1) - cos_theta);

    return rotated_point;
}


template <typename T>Eigen::Matrix<T, 2, 1> project_distort_point(const Eigen::Matrix<T, 3, 1> point_xyz, const Eigen::Matrix<T, 9, 1> camera_params) 
{ 
    /* Take 3d (x,y,z) points in the camera frame and project and disort them into 2d (u,v) distorted image points in the camera frame, using the inputted camera parameters */
    
    // Unpack camera params
    Eigen::Matrix<T, 3, 1> rotation_vec = camera_params.template segment<3>(0);
    Eigen::Matrix<T, 3, 1> translation_vec = camera_params.template segment<3>(3);
    T f = camera_params(6);
    T k1 = camera_params(7);
    T k2 = camera_params(8);

    // Project points onto 2D image plane
    Eigen::Matrix<T, 3, 1> point_proj = rotate_point(point_xyz, rotation_vec); // rotate_points needs to be implemented
    point_proj += translation_vec;
    point_proj(0) = -point_proj(0) / point_proj(2);
    point_proj(1) = -point_proj(1) / point_proj(2);

    // Distort points
    T n = point_proj.squaredNorm();
    T r = T(1) + k1 * n + k2 * n * n;
    Eigen::Matrix<T, 2, 1> point_2d;
    point_2d(0) = point_proj(0) * r * f;
    point_2d(1) = point_proj(1) * r * f;

    return point_2d;
}


struct CostFun {
    /* This cost function seeks to minimize the difference between the observed 2D points in the observed image frame,
    and the 3D points transforme, projected and distorted into the observed image frame. */

    CostFun(const Eigen::Vector2d& point_uv) :  observation_(point_uv) {}
    template <typename T>
    bool operator()(const T* const point_xyz,  // 3D positions of point in camera frame
                    const T* const camera,     // camera parameters
                    T* residual) const {

    // Convert pointers to Eigen matrices
    Eigen::Matrix<T, 3, 1> point(point_xyz[0], point_xyz[1], point_xyz[2]);
    Eigen::Matrix<T, 9, 1> camera_params;
    for (int i = 0; i < 9; ++i) {
        camera_params[i] = camera[i];
    }

    // Project point position from camera frame to world frame
    Eigen::Matrix<T, 2, 1> projection_uv = project_distort_point(point, camera_params);

    // Compare observed and projected point
    residual[0] = projection_uv(0) - observation_(0);
    residual[1] = projection_uv(1) - observation_(1);

    return true;
    }
    const Eigen::Vector2d observation_;
};


int main() {
    // Dataset name (assuming the dataset is already downloaded)
    std::string dataset_name = "../problem-73-11032-pre.txt";

    // Read data from file
    Data data = read_data(dataset_name);

    // Set up optimization problem
    ceres::Problem problem;

    // Define loss function 
    ceres::LossFunction* loss = new ceres::HuberLoss(1.0); // this value could be tuned

    // For each observation 
    for (int obs_num = 0; obs_num < data.num_observations; obs_num++) {

        // Get the camera and point indices for this observation
        int camera_index = data.camera_inds[obs_num];
        int point_index = data.point_inds[obs_num];

        Eigen::Vector2d point_uv = data.points_uv.row(obs_num);
        // std::vector<double> point_uv = data.points_uv[obs_num]; // observed, distorted 2D point in camera frame

        // Get pointers to the camera parameters and 3D point
        double* camera = &data.camera_params[camera_index * 9];  // camera parameters for this camera
        double* point_xyz = &data.points_xyz[point_index * 3];  // 3D location of the observed point
        problem.AddParameterBlock(camera, 9); 
        problem.AddParameterBlock(point_xyz, 3); 

        // Cost function to compare observed and projected/distorted points in the same frame
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFun,
                                                                            2,  // residual size
                                                                            3,  // 3d location of points
                                                                            9  // camera parameters
                                                                            >(new CostFun(point_uv));

        problem.AddResidualBlock(cost_function, loss, point_xyz, camera);
    }

   
    // Set up solver options 
    ceres::Solver::Options options; 
    ceres::Solver::Summary summary;
    bool print_progress = false; 
    options.max_num_iterations = 2500; 
    options.linear_solver_type = ceres::SPARSE_SCHUR; 
    options.minimizer_progress_to_stdout = print_progress;
    options.gradient_tolerance = 1e-10; 
    options.parameter_tolerance = 1e-8; 
    options.function_tolerance = 1e-7;

    // Run the optimization 
    ceres::Solve(options, &problem, &summary);

    // Print results
    if (summary.IsSolutionUsable()) { 
        // Print initial cost 
        double average_reprojection_error_0 = summary.initial_cost / summary.num_residuals; 
        std::cout << "Initial cost: " << summary.initial_cost << std::endl; 
        std::cout << " Average initial cost / num residuals: " << average_reprojection_error_0 << std::endl;

        //  Print final cost 
        double average_reprojection_error = summary.final_cost / summary.num_residuals; 
        std::cout << "Final cost: " << summary.final_cost << std::endl; 
        std::cout << " Average final cost / num residuals: " << average_reprojection_error << std::endl;
        std::cout << summary.FullReport() << std::endl;
    } else { 
        std::cout << "Optimization failed." << std::endl; 
        std::cout << summary.FullReport() << std::endl; 
    }

    return 0;
}