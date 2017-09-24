#include <iostream>
#include <memory>
#include <iomanip>
#include <math.h>
#include "behavior.h"
#include "trajectory.h"
#include "prediction.h"
#include "state.h"
#include "cost.h"

using namespace std;

PossibleTrajectory::PossibleTrajectory(
    shared_ptr<State> fromState,
    shared_ptr<State> toState,
    vector<vector<double>> trajectory,
    vector<vector<double>> sensor_fusion,
    double cost,
    double car_x, double car_y, double car_s, double car_d, double car_yaw,
    shared_ptr<PossibleTrajectory> prev,
    int depth
) {
  fromState_ = fromState;
  toState_ = toState;
  trajectory_ = trajectory;
  sensor_fusion_ = sensor_fusion;
  cost_ = cost;
  depth_ = depth;

  car_x_ = car_x;
  car_y_ = car_y;
  car_s_ = car_s;
  car_d_ = car_d;
  car_yaw_ = car_yaw;

  total_cost_ = cost;

  // set previous possible trajectory
  prev_ = prev;
  if (prev) {
    total_cost_ += prev->total_cost_;
  }

}
PossibleTrajectory::~PossibleTrajectory() {}

void PossibleTrajectory::print(
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y
) {
  Trajectory trajectory;

  cout << "toState: " << left << setw(30) << toState_->name_;
  cout << "Lane Car: " << left << setw(2) << trajectory.getLaneNumber(car_d_);
  cout << "Lane Target: " << left << setw(4) << toState_->target_lane_;
  cout << "Cost: " << left << setw(12) << cost_;

  cout << endl;
}

BehaviorPlanner::BehaviorPlanner() {}
BehaviorPlanner::~BehaviorPlanner() {}

void BehaviorPlanner::transition(
  double car_x,
  double car_y,
  double car_s,
  double car_d,
  double car_yaw,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
  vector<shared_ptr<State>> &states,
  shared_ptr<State> &state,
  bool &thread_is_done
) {

  vector<shared_ptr<PossibleTrajectory>> possible_trajectories = getPossibleTrajectoriesRecursive(
    car_x, car_y, car_s, car_d, car_yaw,
    previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
    sensor_fusion,
    sensor_fusion_history,
    state,
    nullptr,
    4 // depth
  );

  shared_ptr<PossibleTrajectory> lowest = possible_trajectories[0];
  for (int idx = 0; idx < possible_trajectories.size(); idx++) {
    shared_ptr<PossibleTrajectory> current = possible_trajectories[idx];
    if (possible_trajectories[idx]->total_cost_ < lowest->total_cost_) {
      lowest = possible_trajectories[idx];
    }
  }

  cout << "---" << endl;
  states.push_back(lowest->toState_);
  lowest->print(map_waypoints_x, map_waypoints_y);
  while (lowest->lowest_) {
    lowest = lowest->lowest_;
    states.push_back(lowest->toState_);
    lowest->print(map_waypoints_x, map_waypoints_y);
  }

  thread_is_done = true;
}

vector<shared_ptr<PossibleTrajectory>> BehaviorPlanner::getPossibleTrajectoriesRecursive(
  double car_x, double car_y, double car_s, double car_d, double car_yaw,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
  shared_ptr<State> state,
  shared_ptr<PossibleTrajectory> prev,
  int depth
) {

  Trajectory trajectory;
  Predictor predictor;
  map<int, shared_ptr<Prediction>> predictions = predictor.getPredictions(sensor_fusion_history);
  vector<shared_ptr<PossibleTrajectory>> possible_trajectories = BehaviorPlanner::getPossibleTrajectories(
    car_x, car_y, car_s, car_d, car_yaw,
    previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
    sensor_fusion,
    sensor_fusion_history,
    state,
    prev,
    depth
  );

  if (depth > 1) {
    depth = depth - 1;
    for (int i = 0; i < possible_trajectories.size(); i++) {
      // - RECURSION
      int N = trajectory.num_path_;

      previous_path_x = possible_trajectories[i]->trajectory_[0];
      previous_path_y = possible_trajectories[i]->trajectory_[1];

      // last location of the car is right before the the previous path leftovers
      car_x = previous_path_x[N - 3];
      car_y = previous_path_y[N - 3];

      double x1 = previous_path_x[N - 1];
      double y1 = previous_path_y[N - 1];

      double x2 = previous_path_x[N - 2];
      double y2 = previous_path_y[N - 2];

      double car_yaw = atan2(y1 - car_y, x1 - car_x);
      vector<double> sd = trajectory.getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);

      // update sensor fusion to N-2 timestep in the future.
      getPossibleTrajectoriesRecursive(
        car_x, car_y, sd[0], sd[1], car_yaw,
        // previous path contains a couple points so we can get velocity
        {x2, x1},
        {y2, y1},
        map_waypoints_x, map_waypoints_y, map_waypoints_s,
        // future sensor fusion
        predictor.getFutureSensorFusion(
          map_waypoints_x, map_waypoints_y, map_waypoints_s, 
          sensor_fusion, predictions, N - 2
        ),
        sensor_fusion_history,
        possible_trajectories[i]->toState_,
        possible_trajectories[i], // this pointer is modified to link to lowest cost next trajectory
        depth
      );
    }
  } else {

    for (int i = 0; i < possible_trajectories.size(); i++) {
      // propogate lowest total cost trajectory and lowest total cost back to initial state
      shared_ptr<PossibleTrajectory> current = possible_trajectories[i];
      while (current->prev_) {
        if (
          !current->prev_->lowest_
          || (current->total_cost_ < current->prev_->total_cost_)
        ) {
          current->prev_->lowest_ = current;
          current->prev_->total_cost_ = current->total_cost_;
        }
        current = current->prev_;
      }
    }
  } 

  return possible_trajectories;
}

vector<shared_ptr<PossibleTrajectory>> BehaviorPlanner::getPossibleTrajectories(
  double car_x, double car_y, double car_s, double car_d, double car_yaw,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
  shared_ptr<State> fromState,
  shared_ptr<PossibleTrajectory> prev_possible_trajectory,
  int depth
) {

  Trajectory trajectory;
  vector<shared_ptr<PossibleTrajectory>> trajectories;

  // cost functions
  vector<shared_ptr<Cost>> costs = {
    shared_ptr<Cost>(new SlowSpeedCost()),
    shared_ptr<Cost>(new CollideCost()),
    shared_ptr<Cost>(new ChangeLaneCost())
  };

  vector<State::StateId> transitions = fromState->getTransitions();

  for (int t_idx = 0; t_idx < transitions.size(); t_idx++) {

    State::StateId toStateId = transitions[t_idx];

    // find trajectories for each lane
    vector<int> target_lanes = {0, 1, 2};
    for (int l_idx = 0; l_idx < target_lanes.size(); l_idx++) {
      int target_lane = target_lanes[l_idx];

      shared_ptr<State> toState = StateFactory::create(
        toStateId,
        target_lane
      );

      // filter out invalid transitions
      if (!toState->isValid(
        fromState,
        sensor_fusion, sensor_fusion_history,
        car_x, car_y, car_s, car_d, car_yaw,
        previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s
      )) {
        continue;
      };

      // create the trajectory
      vector<vector<double>> possible_trajectory = trajectory.getTrajectory(
        toState,
        sensor_fusion,
        sensor_fusion_history,
        car_x, car_y, car_s, car_d, car_yaw,
        previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
        trajectory.num_path_
      );

      // score trajectory using cost functions
      double cost = 0.;
      for (int c_idx = 0; c_idx < costs.size(); c_idx++) {
        double amount = costs[c_idx]->getCost(
          toState,
          possible_trajectory,
          sensor_fusion,
          sensor_fusion_history,
          map_waypoints_x, map_waypoints_y, map_waypoints_s
        );
        cost += amount;
      }

      // store trajectory
      trajectories.push_back(shared_ptr<PossibleTrajectory>(
        new PossibleTrajectory(
          fromState,
          toState,
          possible_trajectory,
          sensor_fusion,
          cost,
          car_x, car_y, car_s, car_d, car_yaw,
          prev_possible_trajectory,
          depth
        )
      ));
    }
  }

  return trajectories;
}


