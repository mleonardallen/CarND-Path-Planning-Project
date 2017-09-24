#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <memory>
#include "state.h"

class PossibleTrajectory {
 public:

  PossibleTrajectory(
    std::shared_ptr<State> fromState,
    std::shared_ptr<State> toState,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    double cost,
    double car_x, double car_y, double car_s, double car_d, double car_yaw,
    // previous trajectory
    std::shared_ptr<PossibleTrajectory> prev,
    int depth
  );
  virtual ~PossibleTrajectory();
  void print(
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y
  );

  std::shared_ptr<State> fromState_;
  std::shared_ptr<State> toState_;
  std::vector<std::vector<double>> trajectory_;
  std::vector<std::vector<double>> sensor_fusion_;

  double car_x_;
  double car_y_;
  double car_s_;
  double car_d_;
  double car_yaw_;

  double cost_;
  double total_cost_; 

  int depth_;

  // future trajectories branching from self
  std::vector<std::shared_ptr<PossibleTrajectory>> nested_;

  // previous trajectory
  std::shared_ptr<PossibleTrajectory> prev_;

  // lowest total cost future trajectory
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
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
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
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    std::shared_ptr<State> state,
    std::shared_ptr<PossibleTrajectory> prev,
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
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    std::shared_ptr<State> state,
    std::shared_ptr<PossibleTrajectory> prev,
    int depth
  );

 private:

};

#endif /* BEHAVIOR_H */