# rcppm2ros
ROS package for reading four parallel ppm signals from a rc receiver using a Sparkfun Pro Micro.
The Pro Micro runs an Arduino sketch that uses rosserial_arduino to publish messages with the pulse lengths.
The included launch file launches a rosserial_server/serial_node to ralay the messages to the rest of ROS.

To build the arduino sketch run catkin_make install to generate the message files and follow the directions given in http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup to generate the rosserial_arduino libraries.
