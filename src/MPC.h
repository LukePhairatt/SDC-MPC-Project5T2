#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// This value assumes the model presented in the classroom is used.
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Set the timestep length and duration
// Looking too far ahead (twist and turn) might affect the next predict path e.g. off road to accomodate the next turn
// Looking too near distance might not be enough to accomodate twist and turn particularly at high speed 
const size_t N = 10;
const double dt = 0.1;

// Error terms cost factor (this will amplify small errors)
const double etc_weight = 2000.0;             // 2000
const double epsi_weight= 1800.0;             // 1800

// Steering and throttle control cost factor (this will penalise output)
const double steering_penalise = 20.0;        // 20 need less sensity steering to reduce error terms (cte,epsi)      
const double throttle_penalise = 10.0;        // 10
const double steering_rate_penalise = 400.0;  // 400 penalise big change for smoother drives, no big change in steering  
const double throttle_rate_penalise = 20.0;   // 20 (if not penalise much it will break more often, allow big changes)

// Solver variable array indexing
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double target_speed);
  
};

#endif /* MPC_H */
