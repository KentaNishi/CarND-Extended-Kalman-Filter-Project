#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   /* 
   Firstly, this function initialize return variable.
   And then, check the input size.
   Finally, calculate the root mean squere error.
   calculate step is element-wise process.
   */

   VectorXd RMSE(4);
   RMSE << 0, 0, 0, 0;
   
   if (estimations.size() == 0 || estimations.size()!=ground_truth.size())
   {
      cout << "Input is invalid." << endl;
      return RMSE;
   }


   for (int i = 0;i<estimations.size();i++) 
   {
      VectorXd error = estimations[i] - ground_truth[i];
      // to calculate element-wise product
      error = error.array() * error.array();
      RMSE += error;
   }
   RMSE = RMSE/estimations.size();
   RMSE = RMSE.array().sqrt();
   return RMSE;
                              }
   

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

   
  /* I assume that input comes from rader */
  MatrixXd Hj = MatrixXd(3, 4);
  Hj << 0, 0, 0, 0,
	      0, 0, 0, 0,
	      0, 0, 0, 0;
   
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  

  // precompute 

  float c1 = px*px + py*py;

  // TODO: YOUR CODE HERE 
  // check division by zero,the condition is (py==0&&px==0) but too small number is able to be recognize as zero. 
  if (fabs(c1)< 0.0001){
      cout<< "division by zero,so return initial value of Hj" << endl;
      return Hj;
  }

  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  // compute the Jacobian matrix
  Hj <<  px/c2,              py/c2,               0,     0,
        -py/c1,              px/c1,               0,     0,
        py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2, py/c2;

  return Hj;

}
