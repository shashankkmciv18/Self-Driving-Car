# CarND-Capstone
The final project of Udacity Self-Driving Car Engineer
*I had used ros kinetic for this project on Linus 18.04
 
*Earlier i tried to use my own model for both object Detection and Traffic light Classification, but that did not worked.</br>
Then Used YOLO and LeNet(recommended) for the same, which gave me pretty good results


## Project Highlights
* Rewote styx server/bridge with C++ due to terrible performance and instability of original python codes. All ROS nodes can run at 50 Hz after rewriting the server.
* Waypoint updater was coded in C++ too while I was struggling with server's performance issue.
* Used [YOLO](https://github.com/pjreddie/darknet/wiki/YOLO:-Real-Time-Object-Detection) for object detection.
* Used LeNet for traffic light classification


## Furture Improvement
The car always stops earlier than it was directed to.  It seems to be an issue in either DBW node or Waypont Updater node.       
