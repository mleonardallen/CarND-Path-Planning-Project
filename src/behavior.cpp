#include <iostream>
#include <memory>
#include "behavior.h"
#include "trajectory.h"
#include "state.h"
#include "cost.h"

using namespace std;

PossibleTrajectory::PossibleTrajectory(
    shared_ptr<State> state,
    vector<vector<double>> trajectory,
    double cost,
    int target_leading_vehicle_id,
    int target_lane_id
) {
  state_ = state;
  trajectory_ = trajectory;
  cost_ = cost;
  target_leading_vehicle_id_ = target_leading_vehicle_id;
  target_lane_id_ = target_lane_id;
}
PossibleTrajectory::~PossibleTrajectory() {}

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
  shared_ptr<State> &state,
  bool &thread_is_done
) {

  vector<PossibleTrajectory> possible_trajectories = BehaviorPlanner::getPossibleTrajectories(
    car_x, car_y, car_s, car_d, car_yaw,
    previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
    sensor_fusion,
    state
  );

  PossibleTrajectory* lowest = &possible_trajectories[0];
  for (int idx = 1; idx < possible_trajectories.size(); idx++) {
    if (possible_trajectories[idx].cost_ < lowest->cost_) {
      lowest = &possible_trajectories[idx];
    }
  }

  state = lowest->state_;
  thread_is_done = true;
  // cout << " --- planner --- ";
  // cout << "New State: " << state->getName();
  // cout << " Vehicle: " << target_leading_vehicle_id;
  // cout << " Lane: " << target_lane_id << endl;
  // waypoints = possible_trajectories[0].trajectory_;
}

vector<PossibleTrajectory> BehaviorPlanner::getPossibleTrajectories(
  double car_x,double car_y, double car_s, double car_d, double car_yaw,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s,
  vector<vector<double>> sensor_fusion,
  shared_ptr<State> state
) {

  Trajectory trajectory;

  vector<shared_ptr<Cost>> costs = {
    shared_ptr<Cost>(new SlowSpeedCost()),
    shared_ptr<Cost>(new CollideCost()),
    shared_ptr<Cost>(new TooCloseCost())
  };

  vector<PossibleTrajectory> trajectories;
  vector<State::StateId> transitions = state->getTransitions();
  int car_lane = trajectory.getLaneNumber(car_d);

  // get target vehicle ids
  // also append -1 for no target vehicle
  vector<int> target_vehicle_ids;
  target_vehicle_ids.push_back(-1);
  for (int sf_idx = 0; sf_idx < sensor_fusion.size(); sf_idx++) {
    target_vehicle_ids.push_back(sensor_fusion[sf_idx][0]);
  }

  for (int t_idx = 0; t_idx < transitions.size(); t_idx++) {

    for (int v_idx = 0; v_idx < target_vehicle_ids.size(); v_idx++) {

      int target_vehicle_id = target_vehicle_ids[v_idx];

      double diff_s = 0;
      int target_vehicle_lane = car_lane;

      if (target_vehicle_id != -1) { 

        double target_vehicle_s = sensor_fusion[target_vehicle_id][5];
        double target_vehicle_d = sensor_fusion[target_vehicle_id][6];

        diff_s = trajectory.distance(car_s, target_vehicle_s);
        target_vehicle_lane = trajectory.getLaneNumber(target_vehicle_d);
      }

      vector<int> target_lanes = {0, 1, 2};
      for (int l_idx = 0; l_idx < target_lanes.size(); l_idx++) {
        
        shared_ptr<State> transition = StateFactory::create(
          transitions[t_idx],
          target_vehicle_id,
          target_vehicle_lane,
          target_lanes[l_idx]
        );

        // filter out invalid transitions
        if (!transition->isValid(state, car_lane, diff_s)) {
          continue;
        };

        vector<vector<double>> possible_trajectory = trajectory.getTrajectory(
          transition,
          sensor_fusion,
          car_x, car_y, car_s, car_d, car_yaw,
          previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s
        );

        double cost = 0.;
        for (int c_idx = 0; c_idx < costs.size(); c_idx++) {
          cost += costs[c_idx]->getCost(
            transition,
            car_x,
            car_y,
            possible_trajectory,
            sensor_fusion,
            map_waypoints_x,
            map_waypoints_y
          );
        }

        trajectories.push_back(PossibleTrajectory(
          transition,
          possible_trajectory, 
          cost,
          target_vehicle_id,
          target_lanes[l_idx]
        ));
      }
    }
  }

  return trajectories;
}