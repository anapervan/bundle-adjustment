#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

int main() {
    // Download Data

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
    std::vector<std::vector<float>> points_uv(num_observations, std::vector<float>(2));

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

    return 0;
}