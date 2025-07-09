cmake_minimum_required(VERSION 3.0.2)
project(fgo_rio_t)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  nodelet
)

# Boost
find_package(Boost REQUIRED COMPONENTS system serialization filesystem thread date_time regex timer chrono)
include_directories(${Boost_INCLUDE_DIR})
message("Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIR})

#Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

# GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
message("GTSAM_INCLUDE_DIR: " ${GTSAM_INCLUDE_DIR})
message("GTSAM_VERSION: ${GTSAM_VERSION}")


find_package(GTSAM_UNSTABLE REQUIRED)
include_directories(${GTSAM_UNSTABLE_INCLUDE_DIR})
message("GTSAM_UNSTABLE_EINCLUDE_DIR: " ${GTSAM_UNSTABLE_INCLUDE_DIR})
message("GTSAM_UNSTABLE_VERSION: ${GTSAM_UNSTABLE_VERSION}")

add_message_files()
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_generation nodelet
)

add_compile_definitions(GTSAM_USE_BOOST_FEATURES)


include_directories(include   
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME}
        src/fgo_state.cpp
        src/fgo_optimization.cpp
        )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable ${EIGEN3_LIBRARIES})



add_executable(fgo_rio_t_node
        src/odometry.cpp
        ) 
target_link_libraries(fgo_rio_t_node ${PROJECT_NAME} ${catkin_LIBRARIES})


