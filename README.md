# zbig_ws
This code currently uses ROS2 Jazzy for a larger differential drive robot compared to my small differential robot which was modeled after many found on the internet. Much guidance was learned from Udemy courses (instructors: A. Brandi, E. Renard, R. Andrade, & others), ArticulatedRobots videos (J. Newans), and of course the tutorials provided by ROS2, Gazebo, SLAM, MoveIt2, and others.

This code extends the smallbot code to accomodate new function and a > 20 kg payload:
- two Flipky BLDC motors, each using a VESC 
- optical encoding
- large 48V battery with multiple DC-DC converters
- on-board network
- two Arduino Nano 33 iot micros, one for each DC motor. USB and WIFI comm with ROS2.
- one RPi4 using Ubuntu 24.04 & ROS2 Jazzy for robot description and drive
- one RPi5 using Ubuntu 24.04 & ROS2 Jazzy for SLAM

As of 2/1/2025, code is lacking
- navigation
- automation
- image recognition
- multiple manipulators
- payload sense 
