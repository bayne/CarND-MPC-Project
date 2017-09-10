#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;




// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to the center of gravity that has a similar radius.
const double Lf = 2.67;

class FG_eval {
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  double timestep_duration;
  double reference_speed;

  FG_eval(Eigen::VectorXd coeffs) {
    this->coeffs = coeffs;

    /*
     * Have to initialize this for some reason otherwise CppAD complains about symbols not
     * being found
     */
    this->timestep_duration = MPC::kTimestepDuration;
    this->reference_speed = MPC::kReferenceSpeed;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector &fg, const ADvector &vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < MPC::kTimesteps; t++) {
      fg[0] += CppAD::pow(vars[MPC::kCrosstrackErrorStart + t], 2);
      fg[0] += CppAD::pow(vars[MPC::kOrientationErrorStart + t], 2);
      fg[0] += CppAD::pow(vars[MPC::kSpeedStart + t] - reference_speed, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < MPC::kTimesteps - 1; t++) {
      fg[0] += CppAD::pow(vars[MPC::kSteeringAngleStart + t], 2);
      fg[0] += CppAD::pow(vars[MPC::kAcceleratorStart + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < MPC::kTimesteps - 2; t++) {
      fg[0] += CppAD::pow(vars[MPC::kSteeringAngleStart + t + 1] - vars[MPC::kSteeringAngleStart + t], 2);
      fg[0] += CppAD::pow(vars[MPC::kAcceleratorStart + t + 1] - vars[MPC::kAcceleratorStart + t], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + MPC::kXStart] = vars[MPC::kXStart];
    fg[1 + MPC::kYStart] = vars[MPC::kYStart];
    fg[1 + MPC::kOrientationStart] = vars[MPC::kOrientationStart];
    fg[1 + MPC::kSpeedStart] = vars[MPC::kSpeedStart];
    fg[1 + MPC::kCrosstrackErrorStart] = vars[MPC::kCrosstrackErrorStart];
    fg[1 + MPC::kOrientationErrorStart] = vars[MPC::kOrientationErrorStart];

    // The rest of the constraints
    for (int t = 1; t < MPC::kTimesteps; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[MPC::kXStart + t];
      AD<double> y1 = vars[MPC::kYStart + t];
      AD<double> psi1 = vars[MPC::kOrientationStart + t];
      AD<double> v1 = vars[MPC::kSpeedStart + t];
      AD<double> cte1 = vars[MPC::kCrosstrackErrorStart + t];
      AD<double> epsi1 = vars[MPC::kOrientationErrorStart + t];

      // The state at time t.
      AD<double> x0 = vars[MPC::kXStart + t - 1];
      AD<double> y0 = vars[MPC::kYStart + t - 1];
      AD<double> psi0 = vars[MPC::kOrientationStart + t - 1];
      AD<double> v0 = vars[MPC::kSpeedStart + t - 1];
      AD<double> cte0 = vars[MPC::kCrosstrackErrorStart + t - 1];
      AD<double> epsi0 = vars[MPC::kOrientationErrorStart + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[MPC::kSteeringAngleStart + t - 1];
      AD<double> a0 = vars[MPC::kAcceleratorStart + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0;
      AD<double> psides0 = CppAD::atan(coeffs[1]);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * MPC::kTimestepDuration
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * MPC::kTimestepDuration
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * MPC::kTimestepDuration
      // v_[t+1] = v[t] + a[t] * MPC::kTimestepDuration
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * MPC::kTimestepDuration
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * MPC::kTimestepDuration
      fg[1 + MPC::kXStart + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * timestep_duration);
      fg[1 + MPC::kYStart + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * timestep_duration);
      fg[1 + MPC::kOrientationStart + t] = psi1 - (psi0 + v0 * delta0 / Lf * timestep_duration);
      fg[1 + MPC::kSpeedStart + t] = v1 - (v0 + a0 * timestep_duration);
      fg[1 + MPC::kCrosstrackErrorStart + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * timestep_duration));
      fg[1 + MPC::kOrientationErrorStart + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * timestep_duration);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

pair<vector<pair<double, double>>, pair<double, double>> MPC::Solve(const Telemetry & telemetry) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  // place to return solution

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  // Set the number of constraints

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(kModelVariableCount);
  for (i = 0; i < kModelVariableCount; i++) {
    vars[i] = 0;
  }

  // Initialize state
  vars[kXStart] = telemetry.local_x();
  vars[kYStart] = telemetry.local_y();
  vars[kOrientationStart] = telemetry.orientation();
  vars[kSpeedStart] = telemetry.speed();
  vars[kCrosstrackErrorStart] = telemetry.crosstrack_error();
  vars[kOrientationErrorStart] = telemetry.orientation_error();

  Dvector vars_lowerbound(kModelVariableCount);
  Dvector vars_upperbound(kModelVariableCount);

  // No Limits for non actuators
  for (i = 0; i < kSteeringAngleStart; i++) {
    vars_lowerbound[i] = -1e9;
    vars_upperbound[i] = 1e9;
  }

  // Limits for steering angle actuator
  for (i = kSteeringAngleStart; i < kAcceleratorStart; i++) {
    vars_lowerbound[i] = kMinSteeringAngle;
    vars_upperbound[i] = kMaxSteeringAngle;
  }
  
  // Limits for accelerator actuator
  for (i = kAcceleratorStart; i < kModelVariableCount; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(kConstraintCount);
  Dvector constraints_upperbound(kConstraintCount);
  for (i = 0; i < kConstraintCount; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  constraints_lowerbound[kXStart] = telemetry.local_x();
  constraints_lowerbound[kYStart] = telemetry.local_y();
  constraints_lowerbound[kOrientationStart] = telemetry.orientation();
  constraints_lowerbound[kSpeedStart] = telemetry.speed();
  constraints_lowerbound[kCrosstrackErrorStart] = telemetry.crosstrack_error();
  constraints_lowerbound[kOrientationErrorStart] = telemetry.orientation_error();
  
  constraints_upperbound[kXStart] = telemetry.local_x();
  constraints_upperbound[kYStart] = telemetry.local_y();
  constraints_upperbound[kOrientationStart] = telemetry.orientation();
  constraints_upperbound[kSpeedStart] = telemetry.speed();
  constraints_upperbound[kCrosstrackErrorStart] = telemetry.crosstrack_error();
  constraints_upperbound[kOrientationErrorStart] = telemetry.orientation_error();
  

  // object that computes objective and constraints
  FG_eval fg_eval(telemetry.waypoint_model());

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
  std::cout << "status " << solution.status << std::endl;
  std::cout << "Cost " << cost << std::endl;

  vector<pair<double, double>> trajectory;
  if (ok) {
    for (i = 0; i < kTimesteps; i++) {
      trajectory.push_back(
          {
              solution.x[i],
              solution.x[kYStart + i]
          }
      );
    }
    return {
        trajectory,
        {
            -solution.x[kSteeringAngleStart] / kMaxSteeringAngle,
            solution.x[kAcceleratorStart]
        }
    };
  }

  return {
    trajectory,
    {
      0.0,
      0.0
    }
  };

}
