# CarND-Capstone
The final project of Udacity Self-Driving Car Engineer</br>

## Installation, Usage and System Details

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)


## Project Overview
This being the Capstone project required implementation of several topics that were introduced in the course. The objective is to drive a lap on a car first on the simulator and then on Carla (the testing car provided by Udacity) while following traffic signals. The driving route is given using a set of waypoints provided as input. There were primarily 4 areas on which we had to focus:
* Perception
* Planning
* Control
* Integration


## Project Highlights
* Rewote styx server/bridge with C++ due to terrible performance and instability of original python codes. All ROS nodes can run at 50 Hz after rewriting the server.
* Waypoint updater was coded in C++ too while I was struggling with server's performance issue.
* Used [YOLO](https://github.com/pjreddie/darknet/wiki/YOLO:-Real-Time-Object-Detection) for object detection.
* Used LeNet for traffic light classification

## Key Points
* I had used ros kinetic for this project on Linus 18.04
* Earlier i tried to use my own model for both object Detection and Traffic light Classification, but that did not worked.</br>
Then Used YOLO and LeNet(recommended) for the same, which gave me pretty good results</br>
* Other package list can be found in ros/install-ubuntu.sh file


## Furture Improvement
The car always stops earlier than it was directed to.  It seems to be an issue in either DBW node or Waypont Updater node.       
