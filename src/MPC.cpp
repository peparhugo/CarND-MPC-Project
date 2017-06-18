#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
/*
Ths initial values that I tried for the number of time steps and the change in time was 10 and 0,1 respectively. I tuned these parameters along with the tuning constants for the cost function. I found that 10 time steps at 0.1 wasn't plotting a very long trajectory path of the vehicle and I thought this may be causing some of the issues with overcorrecting the steering angle. I reduced the time step to 0.05 and increased the number of steps to 30. This when I had the cost function parameters for the throttle and steering angle set 1000.0 to see if would stabilize the path. It did not.
I left time step to 0.05 and the number of steps to 30 while I tuned the other parameters and ended up getting the vehicle to drive around the track without having to retune these. I did see that it was projecting far enough ahead of the vehicle that it was taking into account up coming turns and following the fitted path quite nicely. Those are other reason I chose to not change the number of time steps or the change in time between time steps.
*/
size_t N = 30;
double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

//set reference values for state variables
/*
Latency is dealt with by capping the maximum speed to 30
It is also dealt with by not putting much weight on the velocity error in the cost function
The vehicle never actually reaches the maximum speed but that's because the weight of the 
cost function is place on the value steering angle and throttle as well as the rate of change of the steering angle and throttle over time.
If either of these two assumptions were broken the model would need to take latency into account as the velocity increases
*/
double ref_v = 30;


//set tuning constants for cost function
/*
These paramters were tuned with a target velocity of 30.
I started with all of these tuning parameters for the cost function at 1.0. I started with the rate of change of the actuator values, TUNE_DELTA_SEQ_DIFF and TUNE_ACC_SEQ_DIFF, since I wanted to start by smoothing the rate of change of the actuators. The vehicle was all over the road and the path was shooting left and right with all the cost parameters at 1.0.

After that I set TUNE_DELTA_SEQ_DIFF to 1000.0 and compiled to code to find the change vehicle trajectory stabilized but the throttle was go and stop, go and stop so I then set TUNE_ACC_SEQ_DIFF to 1000.0. This help stabilize the change to the throttle over time. That being said the program still had initial large steering angles and throttle.

I set TUNE_ACC_VAL to 1000.0 to prevent large changes in values for the throttle. This value was far too large and the car was moving at a snails pace. I set this value to 100.0 and left it there.

I then set TUNE_DELTA_VAL to 1000.0 and this prevented the steering angle from reaching values large enough to take curves. I lowered it to 200.0 but found the vehicles path to be unstable with it making over corrective turns. I then attempted settings it to 350.0 and found this was giving ideal driving behavior.

Once I had the cost parameters tuned to the below settings the vehicle drove around the track. 
*/
const double TUNE_CTE = 1.0;
const double TUNE_EPSI = 1.0;
const double TUNE_VEL = 1.0;
const double TUNE_DELTA_VAL = 350.0;
const double TUNE_ACC_VAL = 100.0;
const double TUNE_DELTA_SEQ_DIFF = 1000.0;
const double TUNE_ACC_SEQ_DIFF = 1000.0;

//limits for non-actuators, cte and detla
const double LIMIT_NON_ACTUATOR  = -1.0E19; //unbound
const double LIMIT_CTE = -4.0;
const double LIMIT_DELTA = -0.436332;
const double LIMIT_ACC = -1.0;
const double LIMIT_CONSTRAINT = 0.0;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }


  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;
    
    //Minize the error values and the difference between the current velocity and target
    for (int t = 0; t < N; t++) {
      fg[0] += TUNE_CTE*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += TUNE_EPSI*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += TUNE_VEL*CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += TUNE_DELTA_VAL*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += TUNE_ACC_VAL*CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += TUNE_DELTA_SEQ_DIFF*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += TUNE_ACC_SEQ_DIFF*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //initialize contraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
    //loop is set at time step 1 so time step 0 is t-1
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

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

      //Model constraints at time t
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9

  //6 states and 2 actuators
  size_t n_vars = 6*N+2*(N-1);
  // TODO: Set the number of constraints
  size_t n_constraints = 6*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values from the current state
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  /*
  The constraints below bound the possible trajectory solutions for the planned trajectory of the vehicle given the current state variables and the ideal fitted path
  This is used to prevent eradict driving behavior
  */

  //non-actuators upper and lower limits
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = LIMIT_NON_ACTUATOR;
    vars_upperbound[i] = fabs(LIMIT_NON_ACTUATOR);
  }

  // lower and upper limits for cte
  for (int i = cte_start; i < cte_start + N; i++) {
    vars_lowerbound[i] = LIMIT_CTE;
    vars_upperbound[i] = fabs(LIMIT_CTE);
  }

  // The upper and lower limits of delta (steering angle)
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = LIMIT_DELTA;
    vars_upperbound[i] = fabs(LIMIT_DELTA);
  }

  // Acceleration/decceleration upper and lower limits (throttle).
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = LIMIT_ACC;
    vars_upperbound[i] = fabs(LIMIT_ACC);
  }

  // Lower and upper limits for the constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = LIMIT_CONSTRAINT;
    constraints_upperbound[i] = LIMIT_CONSTRAINT;
  }

  // object that computes objective and constraints
  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_lowerbound[cte_start] = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[psi_start] = state[2];
  constraints_upperbound[v_start] = state[3];
  constraints_upperbound[cte_start] = state[4];
  constraints_upperbound[epsi_start] = state[5];

  FG_eval fg_eval(coeffs);

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
  options += "Numeric max_cpu_time          0.5\n";

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
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  //vector to store steer angle and throttle result as well as x,y trajectory over N time steps
  vector<double> mpc_result;

  //store delta and a start from solution vector
  mpc_result.push_back(solution.x[delta_start]);
  mpc_result.push_back(solution.x[a_start]);

  //store calculated x, y values used for trajectory in simulator
  for(i=0; i<N; i++){
    mpc_result.push_back(solution.x[x_start+i+1]);
    mpc_result.push_back(solution.x[y_start+i+1]);
  }

  return mpc_result;
}
