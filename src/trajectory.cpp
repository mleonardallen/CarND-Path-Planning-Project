#include "trajectory.h"
#include "spline.h"
#include "util.h"
#include <math.h>
#include <iostream>

using namespace std;

Trajectory::Trajectory() {
  Util util_;
}
Trajectory::~Trajectory() {}

vector<vector<double>> Trajectory::getFutureTrajectory(
  double car_x,
  double car_y,
  double car_s,
  double car_d,
  double car_yaw,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s
) {
  std::vector<double> ptsx;
  std::vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = util_.deg2rad(car_yaw);
  int prev_size = previous_path_x.size();

  // get previous points for spline (previous points bring continuity)
  if (prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // get future points for spline
  vector<int> distances = {30, 60, 90};
  for (int i = 0; i < distances.size(); i++) {
    vector<double> xy = util_.getXY(car_s + distances[i], (lane_center_offset_ + lane_size_ * lane_), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(xy[0]);
    ptsy.push_back(xy[1]);
  }

  // convert points to local space
  for (int i = 0; i < ptsx.size(); i++) {
    vector<double> xy = util_.getLocalSpace(ptsx[i], ptsy[i], ref_x, ref_y, ref_yaw);
    ptsx[i] = xy[0];
    ptsy[i] = xy[1];
  }

  // create the spline
  tk::spline spline;
  spline.set_points(ptsx, ptsy);

  // convert spline to points
  vector<double> x_vals;
  vector<double> y_vals;

  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double N = target_dist / (simulator_cycle_ * ref_vel_ / 2.24);

  double x_add_on = 0;

  for (int i = 1; i <= 50 - prev_size; i++) {

    double x_point = x_add_on + target_x / N;
    double y_point = spline(x_point);

    x_add_on = x_point;

    // rotate back to normal after rotating earlier
    vector<double> xy = util_.getGlobalSpace(x_point, y_point, ref_x, ref_y, ref_yaw);

    x_vals.push_back(xy[0]);
    y_vals.push_back(xy[1]);
  }

  return {x_vals, y_vals};
}

vector<vector<double>> Trajectory::getTrajectory(
  double car_x,
  double car_y,
  double car_s,
  double car_d,
  double car_yaw,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s
) {

  // Build trajector from previous points and future points
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  int prev_size = previous_path_x.size();

  // retain previous trajectory points
  next_x_vals.insert(next_x_vals.end(), previous_path_x.begin(), previous_path_x.end());
  next_y_vals.insert(next_y_vals.end(), previous_path_y.begin(), previous_path_y.end());

  // generate new trajectory points
  vector<vector<double>> future = getFutureTrajectory(
    car_x, car_y, car_s, car_d, car_yaw,
    previous_path_x, previous_path_y,
    map_waypoints_x, map_waypoints_y, map_waypoints_s
  );
  next_x_vals.insert(next_x_vals.end(), future[0].begin(), future[0].end());
  next_y_vals.insert(next_y_vals.end(), future[1].begin(), future[1].end());

  return {next_x_vals, next_y_vals};
}
