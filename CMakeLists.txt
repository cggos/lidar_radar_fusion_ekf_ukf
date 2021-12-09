project(ExtendedKF)

cmake_minimum_required(VERSION 3.5)
add_definitions(-std=c++0x)

include_directories("/usr/include/eigen3")
include_directories(include)

set(SRC_EKF src/main_ekf.cpp 
            src/usagecheck.cpp
            src/tools.cpp
            src/datapoint_ekf.cpp
            src/kalmanfilter.cpp
            src/fusionekf.cpp)

add_executable(extended_kf ${SRC_EKF})

set(SRC_UKF src/tools.cpp
            src/usagecheck.cpp
            src/measurementpredictor.cpp
            src/statepredictor.cpp
            src/stateupdater.cpp
            src/datapoint_ukf.cpp
            src/fusionukf.cpp
            src/main_ukf.cpp)

add_executable(unscented_kf ${SRC_UKF})