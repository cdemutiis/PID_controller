#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * Derivatives for gradient descent
   */
  double Kp_derivative;
  double Kd_derivative;
  double Ki_derivative;

  /*
   * Cumulative sums for the derivatives (over 1000 iterations)
   */
  double Kp_derivative_sum;
  double Kd_derivative_sum;
  double Ki_derivative_sum;

  int iteration;
  int iter_interval;
  int count;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Optimise PID controller's parameters
  */
  void GradientDescent(double alpha);
};

#endif /* PID_H */
