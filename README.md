# MACH1 NAVIGATION PACKAGE

### Overview

The mach1_navigation ROS package provides navigation logic for the Yahboom G1 Tank mobile robot. The package includes a set of ROS nodes that can be used to control the motion of a mobile robot based on user input, such as a joystick controller, sensor data and a map of the environment.
<br>
More information regarding the mach1 robot can be found in the [ Mach1 repository](https://github.com/jrendon102/mach1).


### Installation

To use Mach1 Navigation, you will need to have ROS installed on your system. You can install ROS by following the instructions on the official ROS website: http://wiki.ros.org/ROS/Installation.

Once you have ROS installed, you can install Mach1 Navigation by cloning the repository into your catkin workspace and compiling it:

```
cd ~/catkin_ws/src
git clone https://github.com/<username>/mach1_navigation.git
cd ~/catkin_ws
catkin_make    ### You can also use catkin build
```

### License

Mach1 Navigation is released under the [MIT License](https://opensource.org/license/mit/). Please see the LICENSE file for more details.

### Author & Maintainer

Julian Rendon (julianrendon514@gmail.com)
