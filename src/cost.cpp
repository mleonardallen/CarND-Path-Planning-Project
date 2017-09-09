#include <iostream>
#include <numeric>
#include <memory>
#include "cost.h"
#include "math.h"
#include "trajectory.h"
#include "state.h"

using namespace std;

Cost::Cost() {}
Cost::~Cost() {}

double Cost::getBufferViolations(
  double buffer_s,
  double buffer_d,
  double car_x,
  double car_y,
  std::vector<std::vector<double>> waypoints,
  std::vector<std::vector<double>> sensor_fusion,
  vector<double> maps_x,
  vector<double> maps_y,
  vector<double> maps_s
) {

  Trajectory trajectory;
  vector<double> violations;

  for (int w_idx = 0; w_idx < waypoints[0].size(); w_idx++) {

    // s,d of current waypoint 
    double waypoint_x = waypoints[0][w_idx];
    double waypoint_y = waypoints[1][w_idx];
    vector<double> car_sd = trajectory.getFrenet(waypoint_x, waypoint_y, 0, maps_x, maps_y);
    double car_s = car_sd[0];
    double car_d = car_sd[1];

    for (int sf_idx = 0; sf_idx < sensor_fusion.size(); sf_idx++) {

      double target_vehicle_s = sensor_fusion[sf_idx][5];
      double target_vehicle_d = sensor_fusion[sf_idx][6];

      // ignore cars on the other side of the road
      int target_vehicle_lane = trajectory.getLaneNumber(target_vehicle_d);
      if (!(target_vehicle_lane >= 0 && target_vehicle_lane <= 2)) {
        continue;
      }

      double diff_s = fabs(car_s - target_vehicle_s);
      double diff_d = fabs(car_d - target_vehicle_d);

      // percent is stored so larger violations of buffer space have more weight
      if (diff_s <= buffer_s && car_d <= buffer_d) {

        double percent = 0.;
        percent += (buffer_s - diff_s) / buffer_s;
        percent += (buffer_d - diff_d) / buffer_d;

        violations.push_back(percent);
      }

      // update sensor fusion to 1 timestep in the future.
      sensor_fusion = trajectory.getFutureSensorFusion(maps_x, maps_y, maps_s, sensor_fusion, 1);
    }
  }

  return accumulate(violations.begin(), violations.end(), 0.0);
}

CollideCost::CollideCost() {
  weight_ = 100.;
}
double CollideCost::getCost(
  shared_ptr<State> fromState,
  shared_ptr<State> toState,
  double car_x,
  double car_y,
  std::vector<std::vector<double>> waypoints,
  std::vector<std::vector<double>> sensor_fusion,
  vector<double> maps_x, vector<double> maps_y, vector<double> maps_s
) {

  double buffer_s = 1.5;
  double buffer_d = 0.5;

  double amount = getBufferViolations(
    buffer_s,
    buffer_d,
    car_x,
    car_y,
    waypoints,
    sensor_fusion,
    maps_x,
    maps_y,
    maps_s
  ) > 0. ? 1. : 0.;

  return amount * weight_;
}

TooCloseCost::TooCloseCost() {
  weight_ = 1.;
}
double TooCloseCost::getCost(
  shared_ptr<State> fromState,
  shared_ptr<State> toState,
  double car_x,
  double car_y,
  std::vector<std::vector<double>> waypoints,
  std::vector<std::vector<double>> sensor_fusion,
  vector<double> maps_x, vector<double> maps_y, vector<double> maps_s
) {

  double buffer_s = 1.0;
  double buffer_d = 0.5;

  double amount = getBufferViolations(
    buffer_s,
    buffer_d,
    car_x,
    car_y,
    waypoints,
    sensor_fusion,
    maps_x,
    maps_y,
    maps_s
  );

  return amount * weight_;
}

SlowSpeedCost::SlowSpeedCost() {
  weight_ = 10.;
}
double SlowSpeedCost::getCost(
  shared_ptr<State> fromState,
  shared_ptr<State> toState,
  double car_x,
  double car_y,
  std::vector<std::vector<double>> waypoints,
  std::vector<std::vector<double>> sensor_fusion,
  vector<double> maps_x, vector<double> maps_y, vector<double> maps_s
) {
  Trajectory trajectory;

  double average = trajectory.getAverageVelocity(waypoints);
  double max_vel = trajectory.getMaxVelocity();
  double percent = (max_vel - average) / max_vel;
  double amount = sqrt(percent * percent);

  return amount * weight_;
}

ChangeLaneCost::ChangeLaneCost() {
  weight_ = 0.1;
}
double ChangeLaneCost::getCost(
  shared_ptr<State> fromState,
  shared_ptr<State> toState,
  double car_x,
  double car_y,
  std::vector<std::vector<double>> waypoints,
  std::vector<std::vector<double>> sensor_fusion,
  vector<double> maps_x, vector<double> maps_y, vector<double> maps_s
) {
  double amount = 0.;
  if (
    toState->id_ == State::StateId::PREPARE_LANE_CHANGE_LEFT
    || toState->id_ == State::StateId::PREPARE_LANE_CHANGE_RIGHT
    || toState->id_ == State::StateId::LANE_CHANGE_LEFT
    || toState->id_ == State::StateId::LANE_CHANGE_RIGHT
  ) {
    amount = 1.0;
  }
  return amount * weight_;
}