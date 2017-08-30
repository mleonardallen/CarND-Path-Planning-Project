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
  weight_ = 1.;
}
double CollideCost::getCost(
  shared_ptr<State> state,
  double car_x,
  double car_y,
  std::vector<std::vector<double>> waypoints,
  std::vector<std::vector<double>> sensor_fusion,
  vector<double> maps_x, vector<double> maps_y
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
  ) > 0. ? 1. : 0.;

  return amount * weight_;
}

TooCloseCost::TooCloseCost() {
  weight_ = 3.;
}
double TooCloseCost::getCost(
  shared_ptr<State> state,
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
  weight_ = 1.;
}
double SlowSpeedCost::getCost(
  shared_ptr<State> state,
  double car_x,
  double car_y,
  std::vector<std::vector<double>> waypoints,
  std::vector<std::vector<double>> sensor_fusion,
  vector<double> maps_x,
  vector<double> maps_y
) {
  Trajectory trajectory;

  double x1 = car_x;
  double y1 = car_y;

  vector<double> speeds;
  for (int idx = 0; idx < waypoints[0].size(); idx++) {

    double x2 = waypoints[0][idx];
    double y2 = waypoints[1][idx];
    double distance = trajectory.distance(x1, y1, x2, y2);
    double speed = trajectory.velocity(distance);
    speeds.push_back(speed);

    x1 = x2;
    y1 = y2;
  }

  double average = accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
  double percent = (50 - average) / 50;
  double amount = (percent * percent) * 10;

  return amount * weight_;
}

ChangeLaneCost::ChangeLaneCost() {
  weight_ = 1.;
}
double ChangeLaneCost::getCost(
  shared_ptr<State> state,
  double car_x,
  double car_y,
  std::vector<std::vector<double>> waypoints,
  std::vector<std::vector<double>> sensor_fusion,
  vector<double> maps_x,
  vector<double> maps_y
) {
  double amount = 0;
  return amount * weight_;
}