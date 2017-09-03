//
// Created by Brian Payne on 9/3/17.
//

#include "Telemetry.h"
#include <utility>
#include <vector>
#include <cmath>
#include <exception>

using namespace std;

Telemetry::Telemetry(vector<pair<double, double>> & waypoints, double orientation, double x, double y,
                     double steering_angle, double throttle, double speed
)
    : waypoints_(waypoints),
      orientation_(orientation),
      x_(x),
      y_(y),
      steering_angle_(steering_angle),
      throttle_(throttle),
      speed_(speed)
{}

vector<pair<double, double>> Telemetry::localWaypoints() {
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


vector<pair<pair<double, double>, pair<double, double>>> Telemetry::localSegments() {
  vector<pair<pair<double, double>, pair<double, double>>> segments;
  for (int i = 1; i < localWaypoints().size(); i++) {
    segments.push_back(
        {
            localWaypoints()[i - 1],
            localWaypoints()[i]
        }
    );
  }

  return segments;
}

pair<double, double> Telemetry::localWaypoint(double x) {

  for (pair<pair<double, double>, pair<double, double>> segment : localSegments()) {
    if (segment.first.first < x && segment.second.first >= x) {
      return {
          x,
          (((segment.second.second - segment.first.second) / (segment.second.first - segment.first.first)) * (x - segment.first.first)) + segment.first.second
      };
    }
  }

  throw out_of_range("no way point");

}
