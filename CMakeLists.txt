cmake_minimum_required(VERSION 3.10)  # Minimum version of CMake required
project(BundleAdjustment)  # Name of your project

# Set the C++ standard you wish to use (must be C++11 or later for Ceres)
set(CMAKE_CXX_STANDARD 11)

# If you have other C++ flags you want to use, set them here
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Find the Ceres library
find_package(Ceres REQUIRED)

# If Eigen was not installed as part of Ceres, find it separately
# find_package(Eigen3 REQUIRED)

# Include directories for Ceres and Eigen
include_directories(${CERES_INCLUDE_DIRS}) # ${EIGEN3_INCLUDE_DIR} if Eigen is separate

# Define an executable target called 'bundle_adjustment' from the source file 'bundle_adjustment.cpp'
add_executable(bundle_adjustment bundle_adjustment.cpp)

# Link Ceres and Eigen libraries to your executable
target_link_libraries(bundle_adjustment ${CERES_LIBRARIES}) # Eigen is header-only, no need to link binaries

