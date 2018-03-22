#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;
bool debug_on = false;
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

// Reference speed
double v_ref = 45;

// Marker var start
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
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // Set up the cost function
    fg[0] = 0.0;
    if (debug_on == true){
      std::cout << "DEBUG FG_eval: Checkpoint 1" << std::endl;
    }
    // MPC weights
    double w_cte = 4000;
    double w_epsi = 4000;
    double w_v = 1;
    double w_delta = 5;
    double w_a = 5;
    double d_w_delta = 500;
    double d_w_a = 10;
    // part based on reference state
    for (unsigned int x = 0; x < N; x++){
      fg[0] += w_cte * CppAD::pow(vars[cte_start + x], 2);
      fg[0] += w_epsi * CppAD::pow(vars[epsi_start + x], 2);
      fg[0] += w_v * CppAD::pow(vars[v_start + x] - v_ref, 2);
    }

    // Minimize the use of n_actuators
    for (unsigned int x = 0; x < N - 1; x++){
      fg[0] += w_delta * CppAD::pow(vars[delta_start + x], 2);
      fg[0] += w_a * CppAD::pow(vars[a_start + x], 2);
    }

    // Minimize the value gap between sequential actuatotions
    for (unsigned int x = 0; x < N - 2; x++){
      fg[0] += d_w_delta * CppAD::pow(vars[delta_start + x + 1] - vars[delta_start + x], 2);
      fg[0] += d_w_a * CppAD::pow(vars[a_start + x + 1] - vars[a_start + x], 2);
    }
    if (debug_on == true){
      std::cout << "DEBUG FG_eval: Checkpoint 2" << std::endl;
    }
    // Setup the constraints
    // Initial constraints, shift vars by 1 due to cost function is at fg[0]
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    if (debug_on == true){
      std::cout << "DEBUG FG_eval: Checkpoint 3" << std::endl;
    }
    // Rest of the constraints
    for (unsigned int i = 0; i < N - 1; i++){
      if (debug_on == true){
        std::cout << "DEBUG FG_eval: Checkpoint 4" << std::endl;
      }
      // vars at t
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];
      // Actuations only at time t
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      // vars at t+1
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      AD<double> f0 = coeffs[3] * CppAD::pow(x0,3) + coeffs[2] * CppAD::pow(x0,2) + coeffs[1] * x0 + coeffs[0];
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * CppAD::pow(x0,2) + 2 * coeffs[2] * x0 + coeffs[1]);
      // Model equations
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 - (v0/Lf) * delta0  * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + (v0/Lf) * delta0 * dt);
    }
    if (debug_on == true){
      std::cout << "DEBUG FG_eval: Checkpoint 5" << std::endl;
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

  // Number of vars: n_state = 6; N = timesteps, n_actuators = 2
  size_t n_vars = N * state.size() + (N-1) * 2;
  // Number of constraints
  size_t n_constraints = N * state.size();

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  if (debug_on == true){
    std::cout << "DEBUG SOLVE: Checkpoint 1" << std::endl;
  }
  // Initial states

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];


  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for variables.
  for (unsigned int i = 0; i < delta_start; i++){
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

    // Set lower and upper limit of delta_val [-25, 25] degrees
  for (unsigned int i = delta_start; i < a_start; i++){
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Set lower and upper limit for acceleration
  for (unsigned int i = a_start; i < n_vars; i++){
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  if (debug_on == true){
    std::cout << "DEBUG SOLVE: Checkpoint 2" << std::endl;
  }
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // Lower and upper limits for initial state
  constraints_lowerbound[x_start] = x;
  constraints_upperbound[x_start] = x;

  constraints_lowerbound[y_start] = y;
  constraints_upperbound[y_start] = y;

  constraints_lowerbound[psi_start] = psi;
  constraints_upperbound[psi_start] = psi;

  constraints_lowerbound[v_start] = v;
  constraints_upperbound[v_start] = v;

  constraints_lowerbound[cte_start] = cte;
  constraints_upperbound[cte_start] = cte;

  constraints_lowerbound[epsi_start] = epsi;
  constraints_upperbound[epsi_start] = epsi;


  // object that computes objective and constraints
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
  if (debug_on == true){
    std::cout << "DEBUG SOLVE: Checkpoint 3" << std::endl;
  }
  std::vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  for (unsigned int i = 0; i < N - 1; i++){
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }
  if (debug_on == true){
    std::cout << "DEBUG SOLVE: Checkpoint 4" << std::endl;
  }
  return result;
}
