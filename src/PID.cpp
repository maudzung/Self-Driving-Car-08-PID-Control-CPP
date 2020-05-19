#include <uWS/uWS.h>
#include "PID.h"
#include <math.h>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  // Differential error -> d(cte(t))/dt -> (cte - prev_cte)/dt
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

void PID::RunTwiddle(double err, std::vector<double>& params, std::vector<double>& dps, double& best_err, 
  int& twiddle_pid_index) {
  double tolerance = 0.001;
  double sum_dps = 0.0;


  for (double dp : dps) {
    sum_dps += dp;
  }

  if (sum_dps > tolerance) {
    switch (twiddle_state) {
      case INIT: {
        best_err = err;
        params[twiddle_pid_index] += dps[twiddle_pid_index];
        twiddle_state = INCREASED;
        std::cout << "INCREASED: increase params: " << twiddle_pid_index << std::endl;
        break;
      }
      case INCREASED: {
        if (err < best_err) {
          best_err = err;
          dps[twiddle_pid_index] *= 1.1;
          std::cout << "INCREASED: best, increase 1.1 dps: " << twiddle_pid_index << std::endl;

          // Consider next params
          do {
            twiddle_pid_index = (twiddle_pid_index + 1) % params.size();
          } while (dps[twiddle_pid_index] == 0);

          std::cout << "INCREASED: next twiddle_pid_index: " << twiddle_pid_index << std::endl;

          params[twiddle_pid_index] += dps[twiddle_pid_index];

          std::cout << "INCREASED: increase params: " << twiddle_pid_index << std::endl;
        }
        else {
          params[twiddle_pid_index] -= 2 * dps[twiddle_pid_index];
          twiddle_state = DECREASED;
          std::cout << "INCREASED: decrease params: " << twiddle_pid_index << std::endl;
        }

        break;
      }
      case DECREASED: {
        if (err < best_err) {
          best_err = err;
          dps[twiddle_pid_index] *= 1.1;
          std::cout << "DECREASED: best, increase 1.1 dps: " << twiddle_pid_index << std::endl;
        } 
        else {
          params[twiddle_pid_index] += dps[twiddle_pid_index];
          dps[twiddle_pid_index] *= 0.9;
          std::cout << "DECREASED: increase params, decrease 0.9 dps: " << twiddle_pid_index << std::endl;
        }
        twiddle_state = INCREASED;

        // Consider next params
        do {
          twiddle_pid_index = (twiddle_pid_index + 1) % params.size();
        } while (dps[twiddle_pid_index] == 0);
        std::cout << "DECREASED: next twiddle_pid_index: " << twiddle_pid_index << std::endl;

        params[twiddle_pid_index] += dps[twiddle_pid_index];
        std::cout << "DECREASED: increase params: " << twiddle_pid_index << std::endl;

        break;
      }

    }
  }
  else {
    std::cout<<"Final Kp: " << params[0] << ", Ki: " << params[1] << ", Kd: " << params[2] << std::endl;
  }

}
