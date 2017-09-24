#include <iostream>
#include <numeric>
#include <memory>
#include <iomanip>
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
  vector<double> maps_s,
  bool print
) {

  Trajectory trajectory;
  Predictor predictor;
  vector<double> violations;
  map<int, shared_ptr<Prediction>> predictions = predictor.getPredictions(sensor_fusion_history);

  double car_length_s = 5.0;
  double car_width_d = 3;
  int N = waypoints[0].size();

  for (int w_idx = 0; w_idx < N; w_idx++) {

    vector<double> waypoints_x = waypoints[0];
    vector<double> waypoints_y = waypoints[1];

    double car_x = waypoints_x[w_idx];
    double car_y = waypoints_y[w_idx];

    // first we need car_yaw
    double x1;
    double x2;
    double y1;
    double y2;
    if (w_idx == 0) {
      x1 = car_x;
      y1 = car_y;
      x2 = waypoints_x[w_idx+1];
      y2 = waypoints_y[w_idx+1];
    } else {
      x1 = waypoints_x[w_idx-1];
      y1 = waypoints_y[w_idx-1];
      x2 = car_x;
      y2 = car_y;
    }
    double car_yaw = atan2(y2 - y1, x2 - x1);

    // get s,d from current waypoint
    vector<double> car_sd = trajectory.getFrenet(car_x, car_y, car_yaw, maps_x, maps_y);
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
      double diff_d = car_d - target_vehicle_d;

      // percent is stored so larger violations of buffer space have more weight
      // if (print && target_vehicle_id == 0) {
      //   cout << "id: " << left << setw(4) << target_vehicle_id;
      //   cout << "t: " << left << setw(4) << w_idx;
      //   // cout << "\tcar lane: " << trajectory.getLaneNumber(car_d);
      //   // cout << "\ttarget lane: " << target_vehicle_lane;
      //   cout << "car_s: " << left << setw(10) << car_s;
      //   cout << "target_s: " << left << setw(10) << target_vehicle_s;
      //   // cout << "car_d: " << left << setw(10) << car_d;
      //   // cout << "target_d: " << left << setw(10) << target_vehicle_d;
      //   cout << "diff_s: " << left << setw(10) << diff_s;
      //   cout << "diff_d: " << left << setw(10) << diff_d;
      //   // cout << "target_vel: " << trajectory.velocityVXVY(vx, vy);
      //   cout << endl;
      // }

      if (
        fabs(diff_s) < car_length_s + buffer_s
        && fabs(diff_d) < car_width_d + buffer_d
      ) {

        double percent = 0.;
        percent += (buffer_s + car_length_s - fabs(diff_s)) / (buffer_s + car_length_s);
        percent += (buffer_d + car_width_d - fabs(diff_d)) / (buffer_d + car_width_d);

        violations.push_back(percent);
      }
    }

    // update sensor fusion to 1 timestep in the future.
    sensor_fusion = predictor.getFutureSensorFusion(
      maps_x, maps_y, maps_s,
      sensor_fusion, predictions, 1
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
    maps_x, maps_y, maps_s, false
  ) ? weight_ : 0;
}

bool CollideCost::isCollision(
  vector<vector<double>> waypoints,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
  vector<double> maps_x, vector<double> maps_y, vector<double> maps_s,
  bool print
) {
  return getBufferViolations(
    0, // buffer_s
    0, // buffer_d
    waypoints,
    sensor_fusion,
    sensor_fusion_history,
    maps_x, maps_y, maps_s,
    print
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

  double buffer_s = 5;
  double buffer_d = 0.5;

  double amount = getBufferViolations(
    buffer_s,
    buffer_d,
    waypoints,
    sensor_fusion,
    sensor_fusion_history,
    maps_x,
    maps_y,
    maps_s,
    false
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
  weight_ = 0.3;
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
