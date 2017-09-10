#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Telemetry.h"

using namespace std;

class MPC {
private:
  constexpr static double kMinSteeringAngle = -0.436332 * Telemetry::kLf;
  constexpr static double kMaxSteeringAngle = 0.436332 * Telemetry::kLf;
public:
  const static size_t kTimesteps = 10;
  constexpr static double kTimestepDuration = 0.10;
  constexpr static double kReferenceSpeed = 40;
  const static size_t kXStart = 0;
  const static size_t kYStart = kXStart + kTimesteps;
  const static size_t kOrientationStart = kYStart + kTimesteps;
  const static size_t kSpeedStart = kOrientationStart + kTimesteps;
  const static size_t kCrosstrackErrorStart = kSpeedStart + kTimesteps;
  const static size_t kOrientationErrorStart = kCrosstrackErrorStart + kTimesteps;
  const static size_t kSteeringAngleStart = kOrientationErrorStart + kTimesteps;
  const static size_t kAcceleratorStart = kSteeringAngleStart + kTimesteps - 1;

  const static size_t kModelVariableCount = kTimesteps * Telemetry::STATE_VARIABLE_COUNT + Telemetry::ACTUATOR_COUNT * (kTimesteps - 1);
  const static size_t kConstraintCount = kTimesteps * Telemetry::STATE_VARIABLE_COUNT;


  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  pair<vector<pair<double, double>>, pair<double, double>> Solve(const Telemetry & telemetry);

  constexpr static double crosstrack_error_scale= 2000;
  constexpr static double orientation_error_scale= 2000;
  constexpr static double steering_angle_scale= 5;
  constexpr static double accelerator_scale= 5;
  constexpr static double steering_angle_smooth_scale= 400;
  constexpr static double accelerator_smooth_scale= 10;
//  constexpr static double crosstrack_error_scale= 1.0;
//  constexpr static double orientation_error_scale= 1.0;
//  constexpr static double steering_angle_scale= 1.0;
//  constexpr static double accelerator_scale= 1.0;
//  constexpr static double steering_angle_smooth_scale= 1.0;
//  constexpr static double accelerator_smooth_scale= 1.0;
};

#endif /* MPC_H */
