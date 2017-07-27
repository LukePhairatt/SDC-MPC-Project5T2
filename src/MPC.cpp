#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <vector>

using CppAD::AD;



class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  // Reference speed
  double ref_v;
  FG_eval(Eigen::VectorXd coeffs, double ref_v) 
  { 
    this->coeffs = coeffs; 
    this->ref_v  = ref_v; 
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost function and constraints, 
    // `vars` is a vector of variable values (state & actuators)
    
    
    // -------------------------
    //      Setup Cost
    // -------------------------
    
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the target reference e.g. crosstrack, heading, velocity error 
    for (int t = 0; t < N; t++) {
      fg[0] += etc_weight*CppAD::pow(vars[cte_start + t], 2);          	// target error = 0 
      fg[0] += epsi_weight*CppAD::pow(vars[epsi_start + t], 2);         // target error = 0
      fg[0] += 1.0*CppAD::pow(vars[v_start + t] - ref_v, 2);    		// variable traget speeds ref_v
    }

    // Minimize the use of actuators e.g. steering, throttle
    for (int t = 0; t < N - 1; t++) {
      fg[0] += steering_penalise * CppAD::pow(vars[delta_start + t], 2);        //e.g. sum sq[δ​1​​,....,δ​4​​]
      fg[0] += throttle_penalise * CppAD::pow(vars[a_start + t], 2);            //e.g. sum sq[a1,....,a4]
    }

    // Minimize the value gap between sequential actuations.
    // Amplify the small change in steering, now small change will have a big impact!
    for (int t = 0; t < N - 2; t++) {
      fg[0] += steering_rate_penalise * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);     // to make pair (0,1) (1,2), (2,3), (3,4)...
      fg[0] += throttle_rate_penalise * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);			  // to make pair (0,1) (1,2), (2,3), (3,4)...
    }

    // ------------------------
    // Setup Constraints
    // ------------------------
    
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    // My note: remember fg[0] is a cost
    //                   fg[other] is a constraint
 
    
    fg[1 + x_start] = vars[x_start];           
    fg[1 + y_start] = vars[y_start];           
    fg[1 + psi_start] = vars[psi_start];       
    fg[1 + v_start] = vars[v_start];           
    fg[1 + cte_start] = vars[cte_start];       
    fg[1 + epsi_start] = vars[epsi_start];     

    // The rest of the constraints (start from index 1!)
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // 3rd order polynomial
      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*CppAD::pow(x0, 2) + coeffs[3]*CppAD::pow(x0, 3);
      // My note: getting angle from  atan of slope
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*CppAD::pow(x0, 2));
      
      

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      
      // NOTE: error terms: at tiime t 
      //       cross trck error: cte  = y_predict  - y_actual
      //       orientation error: epsi = psi_actual - psi_predict   
      //       (psi_predict is coming from wp polynomial fitting slope)
      
      //       delta sign term (steering)
      //       simulator steering CW is + and CCW is - (so the model is adjusted to accomodate this)
      //       in opposite delta sign
   
																		  // e.g. if N = 5
                                                                          // e.g. fg[0] is all cost sum square
                                                                          // e.g. fg[1],fg[6],fg[11],fg[21],fg[26] is initial state vars
      fg[1 + x_start + t]    = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);    // e.g. fg[2],[3],[4],[5] constraint between vars(0,1)/(1,2)/(2,3)/(3,4)....
      fg[1 + y_start + t]    = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);    // e.g. fg[7],[8],[9],[10]
      fg[1 + psi_start + t]  = psi1 - (psi0 - v0 * delta0 / Lf * dt);     // e.g. fg[12],[13],[14],[15]
      fg[1 + v_start + t]    = v1 - (v0 + a0 * dt);                       // e.g. fg[17],[18],[19],[20]
      fg[1 + cte_start + t]  =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));             // e.g. fg[22],[23],[24],[25]
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);             // e.g. fg[27],[28],[29],[30]
               
          
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double target_speed) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  // init current state 
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 6 element vector, the actuators is a 2
  // element vector and there are N timesteps. The number of variables is:
  //
  // 6 * N + (N-1) * 2
  const size_t n_vars = N * 6 + (N - 1) * 2;;             			// all variables
  // TODO: Set the number of constraints
  const size_t n_constraints = N * 6;                     			// state vars and errors, but not actuators

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  
  //
  //  Setting state and constraint limits
  //
  // ---------------------------------------------------------
  // vars (state,error,inputs) min-max bound
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // Set all NON-actuators upper and lowerlimits   		 			// e.g. state and errors
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  
  // The upper and lower limits of delta are set to -25 and 25		// e.g. input steering angle 
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332*Lf;
    vars_upperbound[i] = 0.436332*Lf;
  }

  // Acceleration/decceleration upper and lower limits. 			// e.g. input acceleration
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  
  // ---------------------------------------------------------
  // Lower and upper limits for constraints                         // constraints (state,error) min-max bound
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);                    // init all constraints to '0'
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;                              // except the initial starts, set to inital state
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;
  
  
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, target_speed);

  //
  // NOTE: You don't have to worry about these options
  //
  
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          1.0\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  
  // return cost, steering, throttle and mpc predicted points for the simulator
  // format: [cost,steering,throttle,x0,y0,x1,y1,........xn-1,yn-1]
  std::vector<double> mpc_result;

  // push data to mpc result
  mpc_result.push_back(cost);
  mpc_result.push_back(solution.x[delta_start]);
  mpc_result.push_back(solution.x[a_start]);
  // just return all the points, we select them out later
  for(int i=0;i<N;i++){
    mpc_result.push_back(solution.x[x_start+i]);
	mpc_result.push_back(solution.x[y_start+i]);
	
  }
  
  
  return mpc_result;
}
