//
// Created by Brian Payne on 9/3/17.
//

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

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
  Telemetry(
      vector<pair<double, double>> &waypoints,
      double orientation,
      double x,
      double y,
      double steering_angle,
      double throttle,
      double speed
  );

  double orientation() { return orientation_; }
  double globalX() { return x_; }
  double globalY() { return y_; }
  double steering_angle() { return steering_angle_; }
  double throttle() { return throttle_; }
  double speed() { return speed_; }

  const vector<pair<double, double>> localWaypoints();

  pair<double, double> localWaypoint(double x);

  double crosstrack_error() { return crosstrack_error_; }

  double orientation_error() { return orientation_error_; }

};


#endif //MPC_TELEMETRY_H
