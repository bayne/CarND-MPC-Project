//
// Created by Brian Payne on 9/3/17.
//

#include "Telemetry.h"

using namespace std;

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

Telemetry::Telemetry(vector<pair<double, double>> & waypoints, double orientation, double x, double y,
                     double steering_angle, double throttle, double speed, double latency
)
    : waypoints_(waypoints),
      orientation_(orientation),
      x_(x),
      y_(y),
      steering_angle_(steering_angle),
      throttle_(throttle),
      speed_(speed),
      latency_(latency)
{
  auto points_x = Eigen::VectorXd(localWaypoints().size());
  auto points_y = Eigen::VectorXd(localWaypoints().size());

  int i = 0;
  for (pair<double, double> waypoint : localWaypoints()) {
    points_x[i] = waypoint.first;
    points_y[i] = waypoint.second;
    i++;
  }

  waypoint_model_ = polyfit(points_x, points_y, 3);
  crosstrack_error_ = polyeval(waypoint_model_, 0);
  orientation_error_ = -atan(waypoint_model_[1]);

}

// Returns the waypoints mapped to the vehicle-space
const vector<pair<double, double>> Telemetry::localWaypoints() {
  vector<pair<double, double>> local_waypoints;
  transform(
      waypoints_.begin(),
      waypoints_.end(),
      back_inserter(local_waypoints),
      [this](pair<double, double> waypoint) -> pair<double, double> {
        return {
            (waypoint.first - x_) * cos(orientation_) + (waypoint.second - y_) * sin(orientation_),
            -(waypoint.first - x_) * sin(orientation_) + (waypoint.second - y_) * cos(orientation_)
        };
      }
  );


  return local_waypoints;
}
