cmake_minimum_required(VERSION 3.23)
project(SimShip LANGUAGES CXX)

include(CPack)
include(GoogleTest)

find_package(GTest REQUIRED)  # libgtest-dev
find_package(Eigen3 REQUIRED NO_MODULE) # libeigen3-dev
find_package(Doxygen REQUIRED dot OPTIONAL_COMPONENTS mscgen dia) # doxygen
# find_package(JSBSim REQUIRED)

add_subdirectory(targets)

# enable_testing needed on all levels to allow correct test discovery form project root
enable_testing()
