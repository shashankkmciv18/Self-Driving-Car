#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::ArrayXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;
//Constructor and destructor

Tools::Tools() {}

Tools::~Tools() {}

 VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {


   // defualt RMSE
   VectorXd rmse = VectorXd(4);
   rmse << 0, 0, 0, 0;

   // chech the length of estimations and ground_truth is the same
   if(estimations.size() != ground_truth.size()){
      std::cout << "Invalid input for Tools::CalculateRMSE: Lenght of estimations and the ground truth is not equal." << std::endl;
      return rmse;
   }

   ArrayXd sum = ArrayXd(4);
   sum << 0, 0, 0, 0;
   size_t num_estimations = estimations.size();
   for(size_t i = 0; i < num_estimations; i++) {

      sum += (estimations[i] - ground_truth[i]).array().pow(2.0);
   }

   return (sum / num_estimations).sqrt();

}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
   MatrixXd Hjacob(3,4);

   // Initialisation.i.e state
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // intermediate values
   float c1 = std::pow(px, 2.0) + std::pow(py, 2.0);
   float c2 = std::sqrt(c1);
   float c3 = std::pow(c1, 3.0 / 2.0);

   // To ensure non infinity value.
   if (fabs(c1) < 0.0001) {
      std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
      return Hjacob;
   }

   // Computing the jacobian matrix
   Hjacob << (px/c2), (py/c2), 0, 0,
         -(py/c1), (px/c1), 0, 0,
         py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

   return Hjacob;

}

