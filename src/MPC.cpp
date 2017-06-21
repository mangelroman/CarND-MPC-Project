#include "MPC.h"
#include "json.hpp"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include <fstream>

using namespace std;
using json = nlohmann::json;
using CppAD::AD;

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double mph2mps(double x) { return x * 0.44704; }
inline double mps2mph(double x) { return x * 2.2369363; }

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
const double max_steer_angle = deg2rad(25);

size_t N = 20;
double dt = 0.1;
double ref_v_max = mph2mps(120);
double ref_v_min = mph2mps(80);
double ref_r_max = 65;
double ref_r_min = 35;
double weight_cte = 2;
double weight_epsi = 12;
double weight_v = 1;
double weight_delta = 1450;
double weight_a = 1;
double weight_dcte = 1;
double weight_ddelta = 10;
double weight_da = 1;

double vr_m = (ref_v_max - ref_v_min) / (ref_r_max - ref_r_min);
double vr_b = ref_v_min - vr_m * ref_r_min;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
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

  vector<double> coeffs;
  double ref_v;
  double v;

  // Coefficients of the fitted polynomial.
  FG_eval(const vector<double> &coeffs, double ref_v, double v) {
    this->coeffs = coeffs;
    this->ref_v = ref_v;
    this->v = v;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  AD<double> polyeval(AD<double> x) {
    AD<double> result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * CppAD::pow(x, i);
    }
    return result;
  }

  AD<double> dpolyeval(AD<double> x) {
    AD<double> result = 0.0;
    for (int i = 1; i < coeffs.size(); i++) {
      result += coeffs[i] * CppAD::pow(x, i - 1) * i;
    }
    return result;
  }

  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
    for (int t = 0; t < N; t++) {
      fg[0] += weight_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += weight_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += weight_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += weight_delta * v * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += weight_a * CppAD::pow(vars[a_start + t], 2);
      fg[0] += weight_dcte * CppAD::pow(vars[cte_start + t + 1] - vars[cte_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += weight_ddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += weight_da * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 * dt / Lf);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((polyeval(x0) - y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - CppAD::atan(dpolyeval(x0))) + v0 * delta0 * dt / Lf);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(): ptsx(N), ptsy(N), steering_value(0), throttle_value(0) {
}

MPC::~MPC() {}

void MPC::load_config(string filename) {

  ifstream file(filename);

  if (file.is_open()) {
    json j;
    file >> j;

    N = j["N"];
    dt = j["dt"];
    ref_v_max = mph2mps(j["ref_v_max"]);
    ref_v_min = mph2mps(j["ref_v_min"]);
    ref_r_max = j["ref_r_max"];
    ref_r_min = j["ref_r_min"];
    weight_cte = j["weight_cte"];
    weight_epsi = j["weight_epsi"];
    weight_v = j["weight_v"];
    weight_delta = j["weight_delta"];
    weight_a = j["weight_a"];
    weight_dcte = j["weight_dcte"];
    weight_ddelta = j["weight_ddelta"];
    weight_da = j["weight_da"];

    vr_m = (ref_v_max - ref_v_min) / (ref_r_max - ref_r_min);
    vr_b = ref_v_min - vr_m * ref_r_min;

    ptsx.resize(N);
    ptsy.resize(N);

    x_start = 0;
    y_start = x_start + N;
    psi_start = y_start + N;
    v_start = psi_start + N;
    cte_start = v_start + N;
    epsi_start = cte_start + N;
    delta_start = epsi_start + N;
    a_start = delta_start + N - 1;
    steering_value = 0;
    throttle_value = 0;
  }
}

vector<double> MPC::delay(const vector<double> &state,
                          const vector<double> &actuators,
                          double latency) {
  double x   = state[0];
  double y   = state[1];
  double psi = state[2];
  double v   = mph2mps(state[3]);

  double delta = -actuators[0] * max_steer_angle;
  double a     = actuators[1];

  // Advance the state to account for latency
  x = x + v * cos(psi) * latency;
  y = y + v * sin(psi) * latency;
  psi = psi + v * delta * latency / Lf;
  v = v + a * latency;

  return {x, y, psi, mps2mph(v)};
}

bool MPC::solve(const vector<double> &state, const vector<double> &coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  size_t n_vars = 6 * N + 2 * (N - 1);
  size_t n_constraints = 6 * N;

  double x     = state[0];
  double y     = state[1];
  double psi   = state[2];
  double v     = mph2mps(state[3]);
  double cte   = state[4];
  double epsi  = state[5];
  double r     = state[6];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  // Set the initial variable values
  vars[x_start]    = x;
  vars[y_start]    = y;
  vars[psi_start]  = psi;
  vars[v_start]    = v;
  vars[cte_start]  = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -max_steer_angle;
    vars_upperbound[i] = max_steer_angle;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);


  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start]    = x;
  constraints_lowerbound[y_start]    = y;
  constraints_lowerbound[psi_start]  = psi;
  constraints_lowerbound[v_start]    = v;
  constraints_lowerbound[cte_start]  = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start]    = x;
  constraints_upperbound[y_start]    = y;
  constraints_upperbound[psi_start]  = psi;
  constraints_upperbound[v_start]    = v;
  constraints_upperbound[cte_start]  = cte;
  constraints_upperbound[epsi_start] = epsi;

  double ref_v;
  if (r <= ref_r_min) {
    ref_v = ref_v_min;
  }
  else if (r >= ref_r_max) {
    ref_v = ref_v_max;
  }
  else {
    ref_v = vr_m * r + vr_b;
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, ref_v, v);

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
  bool ok = true;
  ok &= (solution.status == CppAD::ipopt::solve_result<Dvector>::success);

  if (ok) {
    // Cost
    auto cost = solution.obj_value;
    cout << "Cost " << cost << endl;

    for (int i = 0; i < N; i++) {
      ptsx[i] = solution.x[x_start + i];
      ptsy[i] = solution.x[y_start + i];
    }

    steering_value = -solution.x[delta_start] / max_steer_angle;
    throttle_value = solution.x[a_start];
  }

  return ok;
}
