# zeus_control

<!-- ![](imgs/diagram.png) -->
<img src="imgs/diagram.png" width="600">

This package contains all the necessary code to operate both the real robot and the gazebo simulation manually.
The drivers for visualization include two USB cameras mounted on pan-tilt, systems, a wide angle camera and a ZED stereo camera.

<img src="imgs/sensors.png" width="600">

The control structure goes from joystick signals to motor speed commands.

<img src="imgs/controller.png" width="400">
<img src="imgs/teleop.png" width="800">


## File structure

#### arduino
This folder contains the arduino code to control the pan tilt system for the cameras. It receives commands through ROS serial and moves accordingly.

#### config
This folder contains multiple configuration files for teleop configuration, sensor parameters, sensor fusion parameters, as well as our interface configuration file in rqt.

<img src="imgs/interface.png" width="800">


#### launch
This folder has the necessary launch files to run the teleop, the sensors and the user interface.

#### src
This folder has all the python files. 

- `odom_publisher.py` and `odom_publisher_sim.py` contain the code developed to publish the rover odometry from wheel encoder measurements. 

- `pan_tilt_joy.py` is a script that subscribes to joy messages and sends position commands to the pan-tilt camera system. 

- `sensor_monitor.py` is not used for the moment, but will help us monitor the sensors' publish rate to make sure they are up and running.

- `twist2motor_cmd.py` subscribes to twist messages and outputs 6 motor commands accordingly. Multiple parameters can be tuned by the user to control the rover. 

<img src="imgs/teleop-param.png" width="600">

## Dependencies
### ROS standard packages
joy
teleop_twist_joy
image_transport_plugins
swri_transform_util
gps_common

### Mapviz
Mapviz is used to show rover's position and orientation on a satellite map. 
To install and use it follow steps here




