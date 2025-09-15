cmake_minimum_required(VERSION 3.0.2)
project(point_mass_model)
add_definitions("-Wall -g")
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        mavros_msgs
        gazebo_msgs
        roscpp
        std_msgs
        nav_msgs
        tf
        message_generation
        )
find_package(Eigen3 REQUIRED)
generate_messages(DEPENDENCIES
        std_msgs
        geometry_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ref_traj
        CATKIN_DEPENDS message_runtime geometry_msgs mavros_msgs roscpp std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        #        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(ref_traj src/PointMassPathSearching.cpp)
target_include_directories(ref_traj PUBLIC include/)
target_link_libraries(ref_traj)

add_executable(test_pm src/test_point_mass_trajectory.cpp)
target_link_libraries(test_pm ref_traj ${catkin_LIBRARIES})
