#include "prediction.h"
#include "trajectory.h"
#include <iostream>

using namespace std;

Prediction::Prediction(
  double acceleration
) {
  acceleration_ = acceleration;
}
Prediction::~Prediction() {}

Predictor::Predictor() {}
Predictor::~Predictor() {}

vector<shared_ptr<Prediction>> Predictor::getPredictions(vector<vector<vector<double>>> sensor_fusion_history) {
  vector<shared_ptr<Prediction>> predictions;
  Trajectory trajectory;

  int count = sensor_fusion_history[0].size();
  for (int i = 0; i < count; i++) {

    double vx0 = sensor_fusion_history[0][i][3];
    double vy0 = sensor_fusion_history[0][i][4];

    double v0 = 0;
    double v1 = 0;
    double average = 0;
    double avg_acc = 0;
    double acceleration = 0;

    for (int h = 0; h < sensor_fusion_history.size(); h++) {
      vector<double> target_vehicle = sensor_fusion_history[h][i];

      // average velocity
      v1 = trajectory.velocityVXVY(target_vehicle[3], target_vehicle[4]);
      average = ((average * h) + v1) / (h + 1);

      // acceleration
      double acc = (v1 - v0) / trajectory.cycle_time_ms_;
      avg_acc = ((avg_acc * h) + acc) / (h + 1);
      v0 = v1;

    }

    cout << avg_acc << endl;
    // double velocity = trajectory.getAverageVelocity({xs, ys});
    // cout << i << ", last: " << velocity << ", average: " << average << endl;
    predictions.push_back(shared_ptr<Prediction>(new Prediction(0.0)));
  }

  return predictions;
}


/**
 * @desc get future sensor fusion
 * 
 * @param {vector<vector<double>>} sensor fusion
 * @return {vector<vector<double>>} sensor fusion
 */
vector<vector<double>> Predictor::getFutureSensorFusion(
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
  int N
) {
  Trajectory trajectory;
  vector<vector<double>> new_sensor_fusion;

  for (int i = 0; i < sensor_fusion.size(); i++) {

    // calculate velocity (assume car is going in s direction)
    double id = sensor_fusion[i][0];
    double x = sensor_fusion[i][1];
    double y = sensor_fusion[i][2];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double s = sensor_fusion[i][5];
    double d = sensor_fusion[i][6];

    int lane = trajectory.getLaneNumber(d);
    if (lane >= 0 && lane <= 2) {
      // get future vehicle_s
      double velocity = trajectory.velocityVXVY(vx, vy);
      s += velocity * trajectory.cycle_time_ms_ * N;

      // get future x,y using s,d
      vector<double> xy = trajectory.getXY(s, d, map_waypoints_x, map_waypoints_y, map_waypoints_s);
      x = xy[0];
      y = xy[1];
    }

    new_sensor_fusion.push_back({id, x, y, vx, vy, s, d});
  }

  return new_sensor_fusion;
}


vector<vector<vector<double>>> Predictor::updateHistory(
  vector<vector<vector<double>>> sensor_fusion_history,
  vector<vector<double>> sensor_fusion
) {

  sensor_fusion_history.push_back(sensor_fusion);
  if (sensor_fusion_history.size() > 10) {
    sensor_fusion_history.erase(sensor_fusion_history.begin());
  }

  return sensor_fusion_history;
}
