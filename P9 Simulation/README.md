# CarND-Capstone
The final project of Udacity Self-Driving Car Engineer

## Team Members
Wen-Jung Tseng   
Hsiu-Ling Chien   
Hsin-Ying Chiu 

## Project Highlights
* Rewote styx server/bridge with C++ due to terrible performance and instability of original python codes. All ROS nodes can run at 50 Hz after rewriting the server.
* Waypoint updater was coded in C++ too while I was struggling with server's performance issue.
* Used [YOLO](https://github.com/pjreddie/darknet/wiki/YOLO:-Real-Time-Object-Detection) for object detection.
* Used LeNet for traffic light classification


## Project Recording
Final recording was uploaded to [Youtube](https://youtu.be/-xH5h-DeAN4)

## Furture Improvement
The car always stops earlier than it was directed to.  It seems to be an issue in either DBW node or Waypont Updater node.       
