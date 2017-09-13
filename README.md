## MPC

Model Predictive Control is a method of solving the vehicle control problem by turning it into an optimization problem. By using an adaptive trajectory provided by MPC, we are able to handle issues such as noisey sensors and actuator latency.

### Simulator

The simulator is a Unity application which sends the controller telemetry data regarding the vehicle's: position, velocity, orientation, steering angle, and throttle. It also provides a set of waypoints that indicate the center of the track. As an added bonus to highlight the strengths of MPC, the simulator also provides a latency between the telemetry data and the actuation.

### Controller

The controller communicates to the simulator over WebSockets with the control data and telemetry data encoded in JSON. The controller utilizes the following:

- C++ (for performance)
- [CppAD](https://www.coin-or.org/CppAD/)
- [Eigen (C++ library)](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [IPOPT](https://www.coin-or.org/Ipopt/documentation/) (the optimizer used)

### Model

At the core of MPC lives a model that describes the behavior of the vehicle by its dependent variables when changes are applied to it's independent variables.

**An example of a model used to describe the vehicle**

![screen shot 2017-09-02 at 2 41 19 pm](https://user-images.githubusercontent.com/712014/30307672-a4cd8134-9734-11e7-9fd1-4ee1f8183b54.png)

The following is the model captured in a format understandable by the optimizer.

```C++
fg[1 + MPC::kXStart + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * timestep_duration);
fg[1 + MPC::kYStart + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * timestep_duration);
fg[1 + MPC::kOrientationStart + t] = psi1 - (psi0 + v0 * delta0 / lf * timestep_duration);
fg[1 + MPC::kSpeedStart + t] = v1 - (v0 + a0 * timestep_duration);
fg[1 + MPC::kCrosstrackErrorStart + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * timestep_duration));
fg[1 + MPC::kOrientationErrorStart + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / lf * timestep_duration);
```

#### Dependent variables (sensor data)

The simulator provides us with following telemetry data which compose up our dependent variables.

- Position: The global position of the vehicle
- Velocity: The speed of the vehicle
- Orientation: The direction the vehicle is pointing
- Crosstrack error: The distance the vehicle is from the center of the track
- Orientation error: The difference between the vehicle's orientation and the trajectory

#### Independent Variables (actuators)

The controls we have over the vehicle are the independent variables in our model

- Steering Angle: Limited to 25 degrees
- Throttle: This includes reversal as a negative value

### Cost function

Since MPC is an optimization problem, a cost function is required. In our use-case the cost function is based on the crosstrack error and orientation error at each step. The crosstrack error is the the distance of the vehicle from the center of the track. The orientation error is the difference between the vehicle's orientation and the direction of the track.

```C++
// Hyper-parameters for tuning the model
constexpr static double crosstrack_error_scale= 2000;
constexpr static double orientation_error_scale= 2000;
constexpr static double steering_angle_scale= 5;
constexpr static double accelerator_scale= 5;
constexpr static double steering_angle_smooth_scale= 400;
constexpr static double accelerator_smooth_scale= 10;

...

fg[0] = 0;

// The part of the cost based on the reference state.
for (int t = 0; t < MPC::kTimesteps; t++) {
  fg[0] += crosstrack_error_scale * CppAD::pow(vars[MPC::kCrosstrackErrorStart + t], 2);
  fg[0] += orientation_error_scale * CppAD::pow(vars[MPC::kOrientationErrorStart + t], 2);
  fg[0] += CppAD::pow(vars[MPC::kSpeedStart + t] - reference_speed, 2);
}

// Minimize the use of actuators.
for (int t = 0; t < MPC::kTimesteps - 1; t++) {
  fg[0] += steering_angle_scale * CppAD::pow(vars[MPC::kSteeringAngleStart + t], 2);
  fg[0] += accelerator_scale * CppAD::pow(vars[MPC::kAcceleratorStart + t], 2);
}

// Minimize the value gap between sequential actuations.
for (int t = 0; t < MPC::kTimesteps - 2; t++) {
  fg[0] += steering_angle_smooth_scale * CppAD::pow(vars[MPC::kSteeringAngleStart + t + 1] - vars[MPC::kSteeringAngleStart + t], 2);
  fg[0] += accelerator_smooth_scale * CppAD::pow(vars[MPC::kAcceleratorStart + t + 1] - vars[MPC::kAcceleratorStart + t], 2);
}
```

A couple scaling factors are applied to the cost function to help tune the eventual output.

### Horizon

MPC uses a finite-horizon to guide the vehicle to a desired trajectory. The finite-horizon is defined by two factors:

- number of timesteps
- duration of timestep

These are two tunable parameters that can result in varying success of the controller.

```C++
  // The number of steps in the future to fit the model to
  const static size_t kTimesteps = 10;
  constexpr static double kTimestepDuration = 0.10;
  constexpr static double kReferenceSpeed = 40;
```

#### Waypoints

The waypoints provided in the telemetry data from the simulator is provided in global coordinates. We use these points to calculate the eventual crosstrack error and the fitted polynomial for the track.

```C++
AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);
```

#### Latency

There is a latency between providing the control commands and the actuation of the vehicle. To counter this I added a prediction pre-processing step to the telemetry data that predicts the vehicles telemetry data for some given timestep.

```C++
// The following are predictions for the given latency
double orientation() const { return -speed_ * steering_angle_  * latency_ / kLf; }
double local_x() const { return speed_ * latency_; }
double local_y() const { return 0; }
double speed() const { return speed_ + throttle_*latency_; }
double crosstrack_error()const { return crosstrack_error_ + speed_ * sin(orientation_error_) * latency_; }
double orientation_error()const { return orientation_error_ - orientation(); }
```

### Result

![result](https://media.giphy.com/media/FVQv6MyKAw5AQ/giphy.gif)

The yellow line is the waypoints provided with the telemetry data. The green line is the trajectory solution guiding the vehicle.

