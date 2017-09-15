#include <iostream>
#include <numeric>
#include <memory>
#include "cost.h"
#include "math.h"
#include "trajectory.h"
#include "prediction.h"
#include "state.h"

using namespace std;

Cost::Cost() {}
Cost::~Cost() {}

double Cost::getBufferViolations(
  double buffer_s,
  double buffer_d,
  vector<vector<double>> waypoints,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
  vector<double> maps_x,
  vector<double> maps_y,
  vector<double> maps_s
) {

  Trajectory trajectory;
  Predictor predictor;
  vector<double> violations;

  double car_length_s = 4.5;
  double car_width_d = 4.0;

  for (int w_idx = 0; w_idx < waypoints[0].size(); w_idx++) {

    // s,d of current waypoint 
    double waypoint_x = waypoints[0][w_idx];
    double waypoint_y = waypoints[1][w_idx];
    vector<double> car_sd = trajectory.getFrenet(waypoint_x, waypoint_y, 0, maps_x, maps_y);

    double car_s = car_sd[0];
    double car_d = car_sd[1];

    for (int sf_idx = 0; sf_idx < sensor_fusion.size(); sf_idx++) {

      double target_vehicle_id = sensor_fusion[sf_idx][0];
      double vx = sensor_fusion[sf_idx][3];
      double vy = sensor_fusion[sf_idx][4];

      double target_vehicle_s = sensor_fusion[sf_idx][5];
      double target_vehicle_d = sensor_fusion[sf_idx][6];

      // ignore cars on the other side of the road
      int target_vehicle_lane = trajectory.getLaneNumber(target_vehicle_d);
      if (!(target_vehicle_lane >= 0 && target_vehicle_lane <= 2)) {
        continue;
      }

      double diff_s = trajectory.distanceS1S2(car_s, target_vehicle_s);
      double diff_d = fabs(car_d - target_vehicle_d);

      // 1) s is measured at front of vehicles
      // 2) simulator cars are 4.5 meters long
      // example: if the target vehicle s is 10, and the car s is 5.5, 
      // then the car is hitting rear of the target car
      bool in_rear_buffer = (diff_s > 0 && diff_s < car_length_s + buffer_s);
      // if the distance is negative, then the target car is behind the car
      // example: if the target vehicle s is 5.5 and the car s is 10
      // then the target car is hitting the rear of the car
      bool in_front_buffer = (diff_s < 0 && fabs(diff_s) < car_length_s + buffer_s);

      // percent is stored so larger violations of buffer space have more weight
      if (
        (in_rear_buffer || in_front_buffer)
        && diff_d < (car_width_d / 2) + buffer_d
      ) {

        double percent = 0.;
        percent += (buffer_s - diff_s) / buffer_s;
        percent += (buffer_d - diff_d) / buffer_d;

        violations.push_back(percent);
      }
    }

    // update sensor fusion to 1 timestep in the future.
    sensor_fusion = predictor.getFutureSensorFusion(
      maps_x, maps_y, maps_s,
      sensor_fusion, sensor_fusion_history, 1
    );
  }

  return accumulate(violations.begin(), violations.end(), 0.0);
}

CollideCost::CollideCost() {
  weight_ = 100.;
}
double CollideCost::getCost(
  shared_ptr<State> toState,
  vector<vector<double>> waypoints,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
  vector<double> maps_x, vector<double> maps_y, vector<double> maps_s
) {
  return isCollision(
    waypoints, sensor_fusion, sensor_fusion_history,
    maps_x, maps_y, maps_s
  ) ? weight_ : 0;
}

bool CollideCost::isCollision(
  vector<vector<double>> waypoints,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
  vector<double> maps_x, vector<double> maps_y, vector<double> maps_s
) {
  return getBufferViolations(
    0, // buffer_s
    0, // buffer_d
    waypoints,
    sensor_fusion,
    sensor_fusion_history,
    maps_x, maps_y, maps_s
  ) > 0;
}

TooCloseCost::TooCloseCost() {
  weight_ = 1.;
}
double TooCloseCost::getCost(
  shared_ptr<State> toState,
  vector<vector<double>> waypoints,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
  vector<double> maps_x, vector<double> maps_y, vector<double> maps_s
) {

  double buffer_s = 2;
  double buffer_d = 2;

  double amount = getBufferViolations(
    buffer_s,
    buffer_d,
    waypoints,
    sensor_fusion,
    sensor_fusion_history,
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
  shared_ptr<State> toState,
  vector<vector<double>> waypoints,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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
  weight_ = 0.15;
}
double ChangeLaneCost::getCost(
  shared_ptr<State> toState,
  vector<vector<double>> waypoints,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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
