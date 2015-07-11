# rcppm2ros
ROS package for reading parallel ppm signals from a RC receiver using a Sparkfun Pro Micro.
The Pro Micro runs a Arduino sketch that uses rosserial_arduino to publish messages with the pulse lengths.
The included launch file launches a rosserial_python/serial_node to relay the messages to the rest of ROS.

The launch file is also used for runtime configuration of the Arduino sketch using ROS parameters.
Currently the only parameter is frame_timeout which decides how long the program should wait for new pulses.

The code reads PPM data from pin D10, D14, D15, and D16, but can easily be expanded to include D8 and D9 by adding them to the rxPins array and increasing rxPinCount to 6.  

# Building
To build the Arduino sketch run catkin_make install to generate the message files and follow the directions given in http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup to generate the rosserial_arduino libraries.

# Roadmap

* Support for more channels
* Channel configuration through parameters
* Building Arduino firmware with Arduino-Cmake
* Support for other targets than Sparkfun Pro Micro (ATMEGA 32u4)
* Support for Spektrum satellites and other serial receivers

