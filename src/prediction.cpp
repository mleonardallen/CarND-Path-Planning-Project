#include "prediction.h"
#include "trajectory.h"
#include <iostream>
#include <map>

using namespace std;

Prediction::Prediction(
  double vehicle_id,
  double average_velocity,
  double average_acceleration
) {
  vehicle_id_ = vehicle_id;
  average_velocity_ = average_velocity;
  average_acceleration_ = average_acceleration;
}
Prediction::~Prediction() {}
void Prediction::print() {
  cout << "id: " << vehicle_id_;
  cout << "\taverage velocity: " << average_velocity_;
  cout << "\taverage acceleration: " << average_acceleration_;
  cout << endl;
}

Predictor::Predictor() {}
Predictor::~Predictor() {}

map<int, shared_ptr<Prediction>> Predictor::getPredictions(vector<vector<vector<double>>> sensor_fusion_history) {
  map<int, shared_ptr<Prediction>> predictions;
  Trajectory trajectory;

  int count = sensor_fusion_history[0].size();
  for (int i = 0; i < count; i++) {

    double v0 = 0;
    double v1 = 0;
    double average_velocity = 0;
    double average_acceleration = 0;
    double acceleration = 0;

    double vehicle_id = sensor_fusion_history[0][i][0];

    int d = sensor_fusion_history[0][i][6];
    int lane = trajectory.getLaneNumber(d);

    if (lane >= 0 && lane <= 2) {
      for (int h = 0; h < sensor_fusion_history.size(); h++) {

        vector<double> target_vehicle = sensor_fusion_history[h][i];

        double vx = target_vehicle[3];
        double vy = target_vehicle[4];

        // average velocity
        v1 = trajectory.velocityVXVY(vx, vy);
        average_velocity = ((average_velocity * h) + v1) / (h + 1);

        // average acceleration
        double acceleration = (v1 - v0) / trajectory.cycle_time_ms_;
        average_acceleration = ((average_acceleration * h) + acceleration) / (h + 1);

        v0 = v1;
      }
    }

    predictions[i] = shared_ptr<Prediction>(new Prediction(
      vehicle_id,
      average_velocity,
      average_acceleration
    ));
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
  map<int, shared_ptr<Prediction>> predictions = getPredictions(sensor_fusion_history);

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
      double velocity = predictions[id]->average_velocity_;
      double acceleration = predictions[id]->average_acceleration_;
      s += trajectory.distanceVAT(velocity, acceleration, trajectory.cycle_time_ms_ * N);

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
  if (sensor_fusion_history.size() > 30) {
    sensor_fusion_history.erase(sensor_fusion_history.begin());
  }

  return sensor_fusion_history;
}
