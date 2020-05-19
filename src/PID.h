#ifndef PID_H
#define PID_H

class PID {
public:
  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  enum TwiddleState { INIT, INCREASED, DECREASED };
  TwiddleState twiddle_state = INIT;

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  void RunTwiddle(double err, std::vector<double>& params, std::vector<double>& dps, double& best_err, 
  int& twiddle_pid_index);

private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

};

#endif  // PID_H