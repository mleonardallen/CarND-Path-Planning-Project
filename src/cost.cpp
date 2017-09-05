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
  vector<double> maps_y
) {

  Trajectory trajectory;
  vector<double> violations;

  for (int sf_idx = 0; sf_idx < sensor_fusion.size(); sf_idx++) {
    double vx = sensor_fusion[sf_idx][3];
    double vy = sensor_fusion[sf_idx][4];
    int target_vehicle_id = sensor_fusion[sf_idx][0];
    double target_vehicle_s = sensor_fusion[sf_idx][5];
    double target_vehicle_d = sensor_fusion[sf_idx][6];

    double velocity = trajectory.velocity(vx, vy);
    
    for (int w_idx = 0; w_idx < waypoints[0].size(); w_idx++) {

      target_vehicle_s += trajectory.distance(velocity);

      double waypoint_x = waypoints[0][w_idx];
      double waypoint_y = waypoints[1][w_idx];

      vector<double> car_sd = trajectory.getFrenet(waypoint_x, waypoint_y, 0, maps_x, maps_y);
      double car_s = car_sd[0];
      double car_d = car_sd[1];

      if (
        car_s <= target_vehicle_s + buffer_s
        && car_s >= target_vehicle_s - buffer_s
        && car_d <= target_vehicle_d + buffer_d
        && car_d >= target_vehicle_d - buffer_d
      ) {
        double diff_s = car_s <= target_vehicle_s ? -buffer_s : buffer_s;
        double diff_d = car_d <= target_vehicle_d ? -buffer_d : buffer_d;
        double percent = 0.;
        percent += (car_s + buffer_s) / (target_vehicle_s + buffer_s);
        percent += (car_d + buffer_d) / (target_vehicle_d + buffer_d);
        violations.push_back(percent);
      }
    }
  }

  return accumulate(violations.begin(), violations.end(), 0.0);
}

CollideCost::CollideCost() {
  weight_ = 1000.;
}
double CollideCost::getCost(
  shared_ptr<State> fromState,
  shared_ptr<State> toState,
  double car_x,
  double car_y,
  std::vector<std::vector<double>> waypoints,
  std::vector<std::vector<double>> sensor_fusion,
  vector<double> maps_x, vector<double> maps_y
) {

  double buffer_s = 1.2;
  double buffer_d = 0.5;

  double amount = getBufferViolations(
    buffer_s,
    buffer_d,
    car_x,
    car_y,
    waypoints,
    sensor_fusion,
    maps_x,
    maps_y
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
  vector<double> maps_x,
  vector<double> maps_y
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
    maps_y
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
  vector<double> maps_x,
  vector<double> maps_y
) {
  Trajectory trajectory;

  double average = trajectory.getAverageVelocity(waypoints);
  double max_vel = trajectory.getMaxVelocity();
  double percent = (max_vel - average) / max_vel;
  double amount = sqrt(percent * percent);

  return amount * weight_;
}

ChangeLaneCost::ChangeLaneCost() {
  weight_ = 0.001;
}
double ChangeLaneCost::getCost(
  shared_ptr<State> fromState,
  shared_ptr<State> toState,
  double car_x,
  double car_y,
  std::vector<std::vector<double>> waypoints,
  std::vector<std::vector<double>> sensor_fusion,
  vector<double> maps_x,
  vector<double> maps_y
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