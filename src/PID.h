#ifndef PID_H
#define PID_H

#include <vector>
#include <string>

class PID {
public:
  std::string name;
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

  unsigned long long steps;
  unsigned long long startup_steps;
  double error;
  double best_error;

  std::vector<double> p;
  std::vector<double> dp;
  unsigned current_index;

  enum {
	  TWIDDLE_START = 0,
	  TWIDDLE_UP = 1,
	  TWIDDLE_DOWN = 2
  } current_state;

  /*
  * Constructor
  */
  PID(std::string);

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

  bool UpdateTwiddle();

  void Recaliberate();
};

#endif /* PID_H */
