#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  p_error = 0;
  i_error = 0;
  d_error = 0;

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

}

void PID::UpdateError(double cte) {
  // check if new error is much different from the previous one
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);
  // return 0.0;  // TODO: Add your total error calc here!
}