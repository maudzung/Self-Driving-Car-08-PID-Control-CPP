#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
 Kp = Kp_;
 Ki = Ki_;
 Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  // Differential error -> d(cte(t))/dt -> (cte - prev_cte)/dt
  // Assume dt = 1
  d_error = cte - p_error;
  // Proportional error -> cte
  p_error = cte;
  // Integral error -> integral(cte(t)*dt) -> sum(cte(t))
  i_error += cte;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return Kp*p_error + Ki*i_error + Kd*d_error;
}