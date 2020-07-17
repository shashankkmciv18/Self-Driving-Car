#include <vector>
#include <iostream>
#include <cmath>
#include "PID.h"
using namespace std;
//Constructor fot the pid declared under header file
PID::PID() {}
PID::~PID() {}

/* It is used for creating this PID as this is an feedback algorithm, it must minimize
itself with time when an correct value of kp, kd and ki is placed here is th
the PID algorithm.*/




void PID::Init(double Kp, double Ki, double Kd) {
  /**
   * Initialising the PID, apart from constructor
   */
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  iter = 0;
}



void PID::UpdateError(double err) {
//Here is propotional error
  p_error = err;
  //Derivative eroor
  d_error = err-p_error;
  //Integral eroor
  i_error =i_error+ err;

  iter += 1;

}

double PID::TotalError( ) {
  /**
   * Calculate and return the total error
   * Feedback kind of. calculating the total error in this and returning it for feedback loops.
   */
  return -Kp * p_error - Kd * d_error - Ki * i_error;  
}