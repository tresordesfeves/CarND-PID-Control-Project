#include "PID.h"
#include <iostream>


/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

    Kp=Kp_;
    Ki=Ki_;
    Kd=Kd_;

    double p_error=0,i_error=0, d_error=0;

}

void PID::Update_Ks(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

    Kp=Kp_;
    Ki=Ki_;
    Kd=Kd_;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error=cte-p_error;
  p_error=cte;
  i_error+=cte;
  //std::cout << "     P_err : "<<p_error<<" I_err : "<<i_error<<" D_err : " << d_error<< std::endl;


  



}

double PID::TotalError() {

  double total_error;
  /**
   * TODO: Calculate and return the total error
   */
  total_error=-(Kp * p_error + Kd * d_error + Ki * i_error);


  return total_error;  // TODO: Add your total error calc here!
}