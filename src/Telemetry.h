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

public:
  static const int STATE_VARIABLE_COUNT = 6;
  static const int ACTUATOR_COUNT = 6;

  Telemetry(
      vector<pair<double, double>> &waypoints,
      double orientation,
      double x,
      double y,
      double steering_angle,
      double throttle,
      double speed
  );

  Eigen::VectorXd waypoint_model() const { return waypoint_model_; }
  double orientation() const { return 0.0; }
  double x() const { return x_; }
  double y() const { return y_; }

  double local_x() const { return 0.0; }
  double local_y() const { return 0.0; }

  double steering_angle() const { return steering_angle_; }
  double throttle() const { return throttle_; }
  double speed() const { return speed_; }

  const vector<pair<double, double>> localWaypoints();

  pair<double, double> localWaypoint(double x);

  double crosstrack_error()const { return crosstrack_error_; }

  double orientation_error()const { return orientation_error_; }

};


#endif //MPC_TELEMETRY_H
