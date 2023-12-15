#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <typeinfo>

template <typename T>Eigen::Matrix<T, 2, 1> project_distort_points(const Eigen::Matrix<T, 4, 1> projection, const Eigen::Matrix<T, 4, 1> distortion, const T* landmark) { /* Take 3d (x,y,z) points in the camera frame and project and disort them into 2d (u,v) distorted image points in the camera frame, using the inputted projection and distortion parameters */
    // Unpack camera intrinsics: projection and distortion parameters T fx = projection[0]; // Focal length x T fy = projection[1]; // Focal length y T cx = projection[2]; // Principal point x T cy = projection[3]; // Principal point y
    T k1 = distortion[0]; T k2 = distortion[1]; T k3 = distortion[2]; T k4 = distortion[3];
    
    // Unpack 3d observed points 
    T x = landmark[0]; 
    T y = landmark[1]; 
    T z = landmark[2];

    // Project and distort the landmark points into the distorted image space 
    T r = ceres::sqrt(x * x + y * y); 
    T th = ceres::atan2(r, z);
    T th2 = th * th; 
    T th4 = th2 * th2; 
    T th6 = th4 * th2; 
    T th8 = th4 * th4; // th^8 
    T thd = th * (T(1) + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
    
    T x_distorted = fx * (thd / r) * x + cx; 
    T y_distorted = fy * (thd / r) * y + cy;

    return Eigen::Matrix<T, 2, 1>(x_distorted, y_distorted);
    }


struct CostFunction {
    /* This cost function seeks to minimize the difference between the observed 2D points in the observed image frame,
    and the 3D points transforme, projected and distorted into the observed image frame. */

    CostFunction(const Eigen::Vector2d& points_uv_1, double& focal_length, Eigen::Vector2d& distortion_coeffs) : 
        observation_(points_uv_1), focal_length_(focal_length), distortion_coeffs_(distortion_coeffs) {}
    template <typename T>
    bool operator()(const T* const landmark_xyz,       // 3d positions of points in landmark frame
                    const T* const translation_xyz,    // translation from observation frame to landmark frame
                    const T* const quaternion_wxyz,    // quaternion from observation frame to landmark frame
                    // const T* const focal_lengths,      // focal lengths (fx, fy)
                    // const T* const principal_centers,  // principal centers (cx, cy)
                    // const T* const distortion,         // distortion parameters (k1, k2, k3, k4)
                    T* residual) const {

    // Create a 4x4 transformation matrix from quaternion and translation
    Eigen::Quaternion<T> quaternion_xyzw(quaternion_wxyz[0], quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3]);
    Eigen::Matrix<T, 3, 1> xyz(translation_xyz[0], translation_xyz[1], translation_xyz[2]);
    Eigen::Matrix<T, 4, 4> G_cam_origin = Eigen::Matrix<T, 4, 4>::Identity();
    G_cam_origin.template block<3, 3>(0, 0) = quaternion_xyzw.toRotationMatrix();
    G_cam_origin.template block<3, 1>(0, 3) = xyz;

    // Project landmark position from landmark frame to observation frame
    Eigen::Matrix<T, 4, 1> proj_landmark_h = G_cam_origin * Eigen::Matrix<T, 4, 1>(landmark_xyz[0], landmark_xyz[1], landmark_xyz[2], T(1.0));
    Eigen::Matrix<T, 3, 1> proj_landmark(proj_landmark_h[0], proj_landmark_h[1], proj_landmark_h[2]);

    // Change variable format
    Eigen::Matrix<T, 4, 1> projection_mat(focal_lengths_(0), focal_lengths_(1), principal_centers_(0), principal_centers_(1));
    Eigen::Matrix<T, 4, 1> distortion_mat(distortion_(0), distortion_(1), distortion_(2), distortion_(3));

    Eigen::Matrix<T, 2, 1> projection_uv = project_distort_points(projection_mat, distortion_mat, proj_landmark.data());

    // Compare in the distorted image space
    residual[0] = projection_uv(0) - observation_(0);
    residual[1] = projection_uv(1) - observation_(1);

    return true;
    }

    const Eigen::Vector2d observation_;
    const double focal_length_;
    const Eigen::Vector2d distortion_coeffs_;

};


int main() {
    // File URL and name (assuming the dataset is already downloaded)
    std::string project_url = "http://grail.cs.washington.edu/projects/bal/data/ladybug/";
    std::string dataset_name = "problem-73-11032-pre.txt.bz2";
    std::string full_url = project_url + dataset_name;

    // Read data from file
    std::ifstream file(dataset_name);

    if (!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
        return 1;
    }

    int num_cameras, num_points, num_observations;
    file >> num_cameras >> num_points >> num_observations;

    std::cout << "num_cameras: " << num_cameras << std::endl;
    std::cout << "num_points: " << num_points << std::endl;
    std::cout << "num_observations: " << num_observations << std::endl;

    std::vector<int> camera_inds(num_observations);
    std::vector<int> point_inds(num_observations);
    std::vector<std::vector<float> > points_uv(num_observations, std::vector<float>(2));

    for (int i = 0; i < num_observations; ++i) {
        file >> camera_inds[i] >> point_inds[i] >> points_uv[i][0] >> points_uv[i][1];
    }

    std::vector<float> camera_params(num_cameras * 9);
    for (int i = 0; i < num_cameras * 9; ++i) {
        file >> camera_params[i];
    }

    std::vector<float> points_xyz(num_points * 3);
    for (int i = 0; i < num_points * 3; ++i) {
        file >> points_xyz[i];
    }

    // Set up optimization problem
    ceres::Problem problem;

    int image_ind = 0; 
    Matrix3Nd init_landmarks_xyz = vo_data[image_ind].landmarks_xyz; // initial estimates of 3d locations of points in landmark frame Eigen::Vector3d init_translation = vo_data[image_ind].translation; // initial estimates of translation from observation frame to landmark frame in xyz Eigen::Vector4d init_rotation = vo_data[image_ind].quaternion; // initial estimates of rotation from observation frame to landmark frame in quaternion
    
    // Define loss function 
    ceres::LossFunction* loss = new ceres::HuberLoss(1.0); // this value could be tuned

    // For each timestep 
    for (const VOData& frame : vo_data) {

        // Variables to be optimized (for each timestep) 
        double* translation_xyz = frame.translationDataPointer(); // translation from observation frame to landmark frame in xyz 
        double* quaternion_wxyz = frame.quaternionDataPointer(); // quaternion vector from observation frame to landmark frame 
        problem.AddParameterBlock(translation_xyz, 3); 
        problem.AddParameterBlock(quaternion_wxyz, 4); 
    
        ceres::LocalParameterization* quaternion_param = new ceres::EigenQuaternionParameterization; // Enforce this 4d vector to adhere to the properties of a quaternion (e.g., norm = 1) 
        problem1.SetParameterization((double*)quaternion_wxyz, quaternion_param);

        // For each observed point
        int num_points = frame.landmarks_xyz.rows();  // number of points observed in this frame
        for (int i = 0; i < num_points; ++i) {
            // Parameters to be kept constant (for each observed point)
            Eigen::Vector2d point_uv_0 = frame.points_uv_0.row(i);  // observed, distorted 2d points in landmark frame
            Eigen::Vector2d point_uv_1 = frame.points_uv_1.row(i);  // observed, distorted 2d points in camera frame

            // Variables to be optimized (for each observed point)
            double* landmark_xyz = frame.landmarkXYZDataPointer(i);  // 3d locations of points in landmark frame
            problem.AddParameterBlock(landmark_xyz, 3);

            // Cost function to compare observed and projected/distorted points in the same frame
            ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunction,
                                                                                2,  // residual size
                                                                                3,  // 3d locations of points
                                                                                3,  // translation landmarks to observation frame
                                                                                4,  // rotation quaternion landmarks to observation frame
                                                                                >(
                new CostFunction(point_uv_0, focal_length, distortion_coeffs));
            problem.AddResidualBlock(cost_function, loss, landmark_xyz,translation_xyz, quaternion_wxyz,);
        }
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
    if (summary1.IsSolutionUsable()) { 
        // Print initial cost 
        double average_reprojection_error_0 = summary1.initial_cost / summary1.num_residuals; 
        std::cout << "Initial cost: " << summary1.initial_cost << std::endl; 

        //  Print final cost 
        double average_reprojection_error = summary1.final_cost / summary1.num_residuals; 
        std::cout << "Final cost: " << summary1.final_cost << std::endl; 
        std::cout << " Average final cost / num residuals: " << average_reprojection_error << std::endl;
        std::cout << summary.FullReport() << std::endl;
    } else { 
        std::cout << "Optimization failed." << std::endl; 
        std::cout << summary1.FullReport() << std::endl; 
    }

    return 0;
}