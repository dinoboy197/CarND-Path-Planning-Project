#ifndef PID_H
#define PID_H

#include <vector>

class PID {
 private:
  // individual and total errors
  double current_cross_track_error_;
  double previous_cross_track_error_;
  double total_cross_track_error_;
  std::vector<double> errors_;

  // pid coefficients
  double Kp_;
  double Ki_;
  double Kd_;

 public:
  PID();

  /*
   * Initialize PID parameters.
   */
  void init(const double parameters[], const bool reset_errors = false);

  /*
   * Get parameters.
   */
  void get_parameters(double parameters[]);

  /*
   * Update the PID error variables given error and return the new control value.
   */
  double compute_control_value(const double error);

  /*
   * Calculate reset, and return the total PID error.
   */
  double get_and_reset_total_error();
};

#endif /* PID_H */
