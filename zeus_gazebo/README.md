# zeus_control

This package contains all the Gazebo related files such as worlds and models.

## File structure

#### launch
This folder contains launch file to start the simulation in gazebo with our own rover

#### models
This folder contains several models for the simulation of a Martian world

<img src="imgs/martian_world.png" width="800">


#### worlds
This folder has our world's definition.

#### src
This folder has all the python files. 

- `odom_publisher.py` and `odom_publisher_sim.py` contain the code developed to publish the rover odometry from wheel encoder measurements. 

- `pan_tilt_joy.py` is a script that subscribes to joy messages and sends position commands to the pan-tilt camera system. 

- `sensor_monitor.py` is not used for the moment, but will help us monitor the sensors' publish rate to make sure they are up and running.

- `twist2motor_cmd.py` subscribes to twist messages and outputs 6 motor commands accordingly. Multiple parameters can be tuned by the user to control the rover. 

<img src="imgs/teleop-param.png" width="400">



