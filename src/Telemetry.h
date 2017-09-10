//
// Created by Brian Payne on 9/3/17.
//

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

#ifndef MPC_TELEMETRY_H
#define MPC_TELEMETRY_H

using namespace std;

class Telemetry {
private:

  vector<pair<double, double>> & waypoints_;
  double orientation_;
  double x_;
  double y_;
  double steering_angle_;
  double throttle_;
  double speed_;
  Eigen::VectorXd waypoint_model_;
  double crosstrack_error_;
  double orientation_error_;
  double latency_;

public:
  static constexpr double kLf = 2.67;
  static const int STATE_VARIABLE_COUNT = 6;
  static const int ACTUATOR_COUNT = 2;

  Telemetry(
      vector<pair<double, double>> &waypoints,
      double orientation,
      double x,
      double y,
      double steering_angle,
      double throttle,
      double speed,
      double latency
  );

  Eigen::VectorXd waypoint_model() const { return waypoint_model_; }
  double orientation() const { return -speed_ * steering_angle_  * latency_ / kLf; }
//  double orientation() const { return orientation_; }
  double x() const { return x_; }
  double y() const { return y_; }

  double local_x() const { return speed_ * latency_; }
  double local_y() const { return 0; }

  double steering_angle() const { return steering_angle_; }
  double throttle() const { return throttle_; }
  double speed() const { return speed_ + throttle_*latency_; }

  const vector<pair<double, double>> localWaypoints();

  pair<double, double> localWaypoint(double x);

  double crosstrack_error()const { return crosstrack_error_ + speed_ * sin(orientation_error_) * latency_; }

  double orientation_error()const { return orientation_error_ - orientation(); }

};


#endif //MPC_TELEMETRY_H
