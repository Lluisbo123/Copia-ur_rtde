# Universal Robots RTDE C++ Interface #
A C++ interface for sending and receiving data to/from a UR robot using the 
[Real-Time Data Exchange (RTDE)](https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/real-time-data-exchange-rtde-guide-22229/)
 interface of the robot. The interface can also by used with python, through the provided python bindings.
 
### Documentation ###
Documentation with installation instructions, examples and API resides at <https://sdurobotics.gitlab.io/ur_rtde/>

If you only want to the use the Python interface, you can install ur_rtde through pip:

    pip install --user ur_rtde

### Motivation ###
No widely available C++ interface that utilizes the RTDE of the UR's existed. Most of the available ROS interfaces lacks a lot of features or are very restricted in terms of control.
This interface is meant to be usable with various robot frameworks, which is why the receive and control interface relies only on STL datatypes. One can choose to convert to STL types or
simply rewrite the control and receive interfaces to the desired datatypes to avoid any overhead. The interface aims to make all the functions on the controller available externally in C++ and Python
with bindings. Finally the plan is to make a more complete ROS interface based on this project.

### Build Status ###
[![build status](https://gitlab.com/sdurobotics/ur_rtde/badges/master/pipeline.svg)](https://gitlab.com/sdurobotics/ur_rtde/commits/master)

### Dependencies ###
*  [Boost](https://www.boost.org/)
*  [pybind11](https://github.com/pybind/pybind11) (Optional)

### Compatible Robots ###

*  All CB-Series from CB3/CB3.1 software 3.3
*  All e-Series

### Compatible Operating Systems ###
Currently tested on:

*  Ubuntu 16.04 (Xenial Xerus)
*  Ubuntu 18.04 (Bionic Beaver)
*  macOS 10.14 (Mojave)
*  Windows 10 Pro x64

### Contact ###
If you have any questions or suggestions to the interface, feel free to contact Anders Prier Lindvig <anpl@mmmi.sdu.dk> or create an issue [here](https://gitlab.com/caro-sdu/ur_rtde/issues).
