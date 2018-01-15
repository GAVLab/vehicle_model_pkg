cmake_minimum_required(VERSION 2.8.3)
project(vehicle_model_pkg)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare a cpp library
add_library(vehicle_model
            src/vehicle_model.cpp
            include/${PROJECT_NAME}/vehicle_model.h)
