# Self-Driving Car Engineer Nanodegree Program

# Overview

This project implements a [PID controller](https://en.wikipedia.org/wiki/PID_controller) to control a car in Udacity's simulator([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)). The simulator sends cross-track error, speed and angle to the PID controller(PID) using [WebSocket](https://en.wikipedia.org/wiki/WebSocket) and it receives the steering angle ([-1, 1] normalized) and the throttle to drive the car. The PID uses the [uWebSockets](https://github.com/uNetworking/uWebSockets) WebSocket implementation. Udacity provides a seed project to start from on this project ([here](https://github.com/udacity/CarND-PID-Control-Project)).

# Dependencies

The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

For instructions on how to install these components on different operating systems, please, visit [Udacity's seed project](https://github.com/udacity/CarND-PID-Control-Project).

In order to install the necessary libraries in mac, use the [install-mac.sh](./install-mac.sh).

In order to install the necessary libraries in ubuntu, use the [install-ubuntu.sh](./install-ubuntu.sh).


## PID Controller

Given a driving simulator which provides the cross track error (CTE) and the velocity (mph), the goal of this project is to develop a PID controller in c++ that successfully drives the vehicle around the track. In order to navigate the car around the track. I have implemented 2 PID controllers, one for steering angle control and the other for speed control.

## Tuning


It is the most important and time consuminng part of the whole procedure.I had done manual tuning which is often not considered to be best method for the purpose but still.




#### Proportional Factor (P): 

This parameter controls the error proportionally. 
Increasing P too much always leads to an periodic osscilation thus for tuning PID we always first tune P to such an extend that the Oscillations are just about to start and not yet started howver it is very difficult and requires time.

#### Integral (I): 
This can be considered as the sum of all eroor during the whole procedure.

#### Derivative (D): 
It is rate of change of eroor.


I started initially with (0.1,0.03,2.0) for the steering control and (0.1,0.0,0.0) for speed control. The final parameters for my controllers are: 
[//]: # (Image References)

[image1]: ./output/c2.PNG "PID graph when it is not working propery;"

[image2]: ./output/Capture.PNG "PID graph when it is  working propery;"

[image3]: ./output/1.PNG "PID graph when it is  working propery;"

* This is where we need to tune our parameters
![alt text][image3]

|   | Steering  | Speed  |
|---|---|---|
| Kp  |  0.135 |  0.109 |
| Ki  | 0.00  |  0.0024 |
| Kd  | 1.01  |  0.0 |


---
![alt text][image1]

![alt text][image2]

---
Here's a [link to my video result](./output/pid-controls.mp4)
