zeus_rover

Doxygen documentation : https://robotique-udes.github.io/zeus_rover/

Installation :
```
cd <your_catkin_ws>/src
git clone https://github.com/robotique-udes/zeus_rover.git
cd zeus_rover
catkin_make
```

To use the husky simulation :

Install dependencies
```
sudo apt-get install ros-melodic-realsense2*
```

Launch simulator and robot model
```
roslaunch zeus_gazebo husky.launch
```

In another terminal, launch control nodes and interface
```
roslaunch zeus_control teleop.launch
```

The robot can be controlled using the teleop_twist_keyboard
```
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
```