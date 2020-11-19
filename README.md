# MoveIt_StandAlone

## Prepare Environment

1. *term$* `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`  
Note: Here you are adding the ROS repos but not actually installing ROS

1. *term$* `sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`

1. *term$* `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'`

1. *term$* `wget http://packages.ros.org/ros.key -O - | sudo apt-key add -`

1. *term$* `sudo apt update`

1. *term$* `sudo apt install ros-melodic-moveit-core ros-melodic-moveit-kinematics ros-melodic-moveit-msgs ros-melodic-xacro rosbash ros-melodic-tf-conversions ros-melodic-roscpp ros-melodic-cmake-modules ros-melodic-trac-ik-kinematics-plugin python-catkin-tools liburdf0d liburdf-dev liburdfdom-headers-dev liborocos-kdl1.3 liborocos-kdl-dev libkdl-parser0d libkdl-parser-dev`

1. *term$* `sudo apt install libeigen-stl-containers-dev libgeometric-shapes-dev`

1. Download Eigen3: http://eigen.tuxfamily.org/index.php?title=Main_Page  
(Then navigate to the director to where it was downloaded)
    1. *term$* `mkdir build && cd $_`
    1. *term$* `cmake ..`
    1. *term$* `sudo make install`
  
1. *term$* `echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/melodic/lib' >> ~/.bashrc`

1. *term$* `echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc`

## Build MoveIt! ROS Package
1. *term$* `git clone --recurse-submodules --remote-submodules https://github.com/correlllab/MoveIt_StandAlone.git`  
(This repo)

1. *term$* `cd MoveIt_StandAlone`

1. *term$* `catkin build`
