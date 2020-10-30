cmake_minimum_required(VERSION 2.8)
project(moveittest)
add_definitions(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
SET( GCC_PTHREAD_FLAGS "-lpthread -pthread" ) # This is needed for LIBIGL when using 'catkin_make' , works with plain cmake/make
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_PTHREAD_FLAGS}" ) # Append the flags                 ^^^-- *shrug* ---^^^

list(APPEND CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH})
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic") # This saves a lot of `find_package`s

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

find_package(urdf REQUIRED)
include_directories(SYSTEM ${URDF_INCLUDE_DIRS})

set( ROS_LIBS /opt/ros/melodic/lib )
# link_directories( ${ROS_LIBS} )

set( MOVEIT_LIBS 
  ${ROS_LIBS}/libmoveit_kinematics_plugin_loader.so.1.0.6
  ${ROS_LIBS}/libmoveit_robot_model_loader.so.1.0.6 
  ${ROS_LIBS}/libmoveit_robot_model.so.1.0.6
  ${ROS_LIBS}/libmoveit_profiler.so.1.0.6
  ${ROS_LIBS}/libmoveit_rdf_loader.so.1.0.6
  ${ROS_LIBS}/libmoveit_kinematics_base.so.1.0.6
  ${ROS_LIBS}/libclass_loader.so
)


include_directories( /opt/ros/melodic/include/ ) 
include_directories( src ) # ------- Project specific sources

set( EXEC_NAME test1 )
add_executable( ${EXEC_NAME} src/moveit_test_01.cpp )

set( EXEC_NAME test2 )
add_executable( ${EXEC_NAME} src/moveit_test_02.cpp )
target_link_libraries( ${EXEC_NAME} urdf )

set( EXEC_NAME test3 )
add_executable( ${EXEC_NAME} src/moveit_test_03.cpp )
target_link_libraries( ${EXEC_NAME} ${MOVEIT_LIBS} )


#### SPARE PARTS ##################################################################################

# find_package( catkin REQUIRED PATHS /opt/ros/melodic/share/catkin )
# include_directories(SYSTEM ${CATKIN_INCLUDE_DIRS})

# find_package( genmsg REQUIRED PATHS /opt/ros/melodic/share/genmsg )
# include_directories(SYSTEM ${GENMSG_INCLUDE_DIRS})

# find_package( actionlib_msgs REQUIRED PATHS /opt/ros/melodic/share/actionlib_msgs )
# include_directories(SYSTEM ${ACTIONLIB_MSGS_INCLUDE_DIRS})

# find_package( moveit_msgs REQUIRED PATHS /opt/ros/melodic/share/moveit_msgs )
# include_directories(SYSTEM ${MOVEIT_MSGS_INCLUDE_DIRS})

# find_package( moveit_core REQUIRED )
# include_directories(SYSTEM ${MOVEIT_CORE_INCLUDE_DIRS})

# find_package( eigen_stl_containers REQUIRED PATHS /usr/share/eigen_stl_containers/ )
# include_directories(SYSTEM ${EIGEN_STL_CONTAINERS_INCLUDE_DIRS})

# find_package( geometric_shapes REQUIRED PATHS /usr/share/geometric_shapes/cmake )
# include_directories(SYSTEM ${GEOMETRIC_SHAPES_INCLUDE_DIRS})
