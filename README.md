seeker Node for ROS

Simple node that seeks out an object using laserscan data, and moves toward it.



Subscribed Topics:

/scan



Published Topics:

/displacement - displacement vector of type geometry_msgs/Vector3, shows displacement of object relative to scan frame

/mobile_base/commands/velocity


Services:

/enable - std_srcs/SetBool service to enable/disable the robot


Dependencies:

geometry_msgs

sensor_msgs

std_msgs

std_srvs

roscpp



Installation:

install with catkin_make


Usage:

launch node: $ roslaunch seeker interview.launch

launch simulator: roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="<path>/mini_world.world"

enable: $ rosservice call /enable "data: true"

disable: $ rosservice call /enable "data: false"


Notes:

Node was written and tested using the turtlebot package, and simulated in gazebo.

Turtlebot 2 was used in simulation/testing.


Bugs:

In testing, it was found that the simulated scan data coming in from /scan would not return centered values when the object reached a certain maximum distance, which was less than the reported range of the sensor. This may be a bug with the node, although echoing the /scan topic clearly shows that data is not coming in through the simulated sensor, and it seems to have a "blind" spot after a certain distance.


