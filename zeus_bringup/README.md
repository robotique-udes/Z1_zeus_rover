To launch a launch file on startup, use robot_upstart package. Use install script to create service : http://docs.ros.org/en/jade/api/robot_upstart/html/install.html


to create service
rosrun robot_upstart install --master http://192.168.1.XXX:11311 --job zeus_bringup zeus_bringup/launch/minimal.launch

to remove service
rosrun robot_upstart uninstall zeus_bringup

in usr/sbin/zeus_bringup-start, comment 
#export ROS_HOSTNAME=$(hostname)
and add 
export ROS_IP=192.168.1.XXX
right under

to update the launchfile, remove the service and reinstall it.