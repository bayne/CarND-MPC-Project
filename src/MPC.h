#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Telemetry.h"

using namespace std;

class MPC {
private:
  // 0.436332 is 25 degrees in radians
  constexpr static double kMinSteeringAngle = -0.436332 * Telemetry::kLf;
  constexpr static double kMaxSteeringAngle = 0.436332 * Telemetry::kLf;
public:
  // The number of steps in the future to fit the model to
  const static size_t kTimesteps = 10;
  constexpr static double kTimestepDuration = 0.10;
  constexpr static double kReferenceSpeed = 40;

  // The following are offset values for the vector used in the optimization step
  const static size_t kXStart = 0;
  const static size_t kYStart = kXStart + kTimesteps;
  const static size_t kOrientationStart = kYStart + kTimesteps;
  const static size_t kSpeedStart = kOrientationStart + kTimesteps;
  const static size_t kCrosstrackErrorStart = kSpeedStart + kTimesteps;
  const static size_t kOrientationErrorStart = kCrosstrackErrorStart + kTimesteps;
  const static size_t kSteeringAngleStart = kOrientationErrorStart + kTimesteps;
  const static size_t kAcceleratorStart = kSteeringAngleStart + kTimesteps - 1;

  // Total number of variables in the vector
  const static size_t kModelVariableCount = kTimesteps * Telemetry::STATE_VARIABLE_COUNT + Telemetry::ACTUATOR_COUNT * (kTimesteps - 1);
  const static size_t kConstraintCount = kTimesteps * Telemetry::STATE_VARIABLE_COUNT;


  MPC();

  virtual ~MPC();

  // Solve the model given the telemetry
  // Return the first actuations.
  pair<vector<pair<double, double>>, pair<double, double>> Solve(const Telemetry & telemetry);

  // Hyper-parameters for tuning the model
  constexpr static double crosstrack_error_scale= 2000;
  constexpr static double orientation_error_scale= 2000;
  constexpr static double steering_angle_scale= 5;
  constexpr static double accelerator_scale= 5;
  constexpr static double steering_angle_smooth_scale= 400;
  constexpr static double accelerator_smooth_scale= 10;
};

#endif /* MPC_H */
