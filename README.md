# PAWS Quadruped Robot

## About The Project
ROS driver for a custom built 3D printed quadrupedal robot (named PAWS). This repo contains ROS1 packages for low level motor control, gazebo simuations, whole body kinematics and a basic gait generator.

## Getting Started
The driver has been tested on Ubuntu 20.04 with ROS2 Noetic.

### Prerequisites
* ROS Noetic
* Numpy
* Scipy 

### Installation
To install the PAWS driver:

```
mkdir paws_ws && cd paws_ws
mkdir src && cd src
git clone https://github.com/kousheekc/PAWS-Quadruped-Robot.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Usage
Run the simuation using:
```
roslaunch paws_gait walk.launch
```

## Contact
Kousheek Chakraborty - kousheekc@gmail.com

Project Link: [https://github.com/kousheekc/PAWS-Quadruped-Robot.git](https://github.com/kousheekc/PAWS-Quadruped-Robot.git)

