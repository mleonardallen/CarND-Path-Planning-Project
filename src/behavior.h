#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <memory>
#include "state.h"

class PossibleTrajectory {
 public:

  PossibleTrajectory(
    std::shared_ptr<State> state,
    std::vector<std::vector<double>> trajectory,
    double cost,
    int target_lane_id,
    int target_leading_vehicle_id
  );
  virtual ~PossibleTrajectory();

  std::shared_ptr<State> state_;
  std::vector<std::vector<double>> trajectory_;

  double cost_;
  double total_cost_;
  double future_cost_;

  int target_lane_id_;
  int target_leading_vehicle_id_;

  std::vector<std::shared_ptr<PossibleTrajectory>> nested_;
  std::shared_ptr<PossibleTrajectory> prev_;
  std::shared_ptr<PossibleTrajectory> lowest_;
};

class BehaviorPlanner {
 public:
  BehaviorPlanner();
  virtual ~BehaviorPlanner();

  static void transition(
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::shared_ptr<State>> &states,
    std::shared_ptr<State> &state,
    bool &thread_is_done
  );

  static std::vector<std::shared_ptr<PossibleTrajectory>> getPossibleTrajectoriesRecursive(
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s,
    std::vector<std::vector<double>> sensor_fusion,
    std::shared_ptr<State> state,
    int depth
  );

  static std::vector<std::shared_ptr<PossibleTrajectory>> getPossibleTrajectories(
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s,
    std::vector<std::vector<double>> sensor_fusion,
    std::shared_ptr<State> state
  );

  static std::vector<std::vector<double>> getFutureSensorFusion(
    int N,
    std::vector<double> maps_x,
    std::vector<double> maps_y,
    std::vector<double> maps_s,
    std::vector<std::vector<double>> sensor_fusion
  );

 private:

};

#endif /* BEHAVIOR_H */