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
    int target_lane_id,
    shared_ptr<PossibleTrajectory> prev
) {
  state_ = state;
  trajectory_ = trajectory;
  cost_ = cost;
  total_cost_ = cost;
  target_leading_vehicle_id_ = target_leading_vehicle_id;
  target_lane_id_ = target_lane_id;

  // set previous possible trajectory
  prev_ = prev;
  if (prev) {
    total_cost_ += prev->total_cost_;
  }

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
  vector<shared_ptr<State>> &states,
  shared_ptr<State> &state,
  bool &thread_is_done
) {

  vector<shared_ptr<PossibleTrajectory>> possible_trajectories = getPossibleTrajectoriesRecursive(
    car_x, car_y, car_s, car_d, car_yaw,
    previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
    sensor_fusion,
    state,
    nullptr,
    3 // depth
  );

  shared_ptr<PossibleTrajectory> lowest = possible_trajectories[0];
  for (int idx = 1; idx < possible_trajectories.size(); idx++) {
    if (possible_trajectories[idx]->total_cost_ < lowest->total_cost_) {
      lowest = possible_trajectories[idx];
    }
  }

  states.push_back(lowest->state_);
  while (lowest->lowest_) {
    lowest = lowest->lowest_;
    states.push_back(lowest->state_);
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
  shared_ptr<State> state,
  shared_ptr<PossibleTrajectory> prev,
  int depth
) {

  vector<shared_ptr<PossibleTrajectory>> possible_trajectories = BehaviorPlanner::getPossibleTrajectories(
    car_x, car_y, car_s, car_d, car_yaw,
    previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
    sensor_fusion,
    state,
    prev
  );

  if (depth > 1) {
    Trajectory trajectory;
    depth = depth - 1;
    for (int i = 0; i < possible_trajectories.size(); i++) {
      // - RECURSION
      vector<double> xs = possible_trajectories[i]->trajectory_[0];
      vector<double> ys = possible_trajectories[i]->trajectory_[1];
      double future_x = xs.back();
      double future_y = ys.back();
      vector<double> future_sd = trajectory.getFrenet(future_x, future_y, 0, map_waypoints_x, map_waypoints_y);

      int N = possible_trajectories[i]->trajectory_[0].size();
      vector<vector<double>> future_sensor_fusion = getFutureSensorFusion(N, map_waypoints_x, map_waypoints_y, map_waypoints_s, sensor_fusion);

      getPossibleTrajectoriesRecursive(
        future_x,
        future_y,
        future_sd[0],
        future_sd[1],
        car_yaw,
        possible_trajectories[i]->trajectory_[0],
        possible_trajectories[i]->trajectory_[1],
        map_waypoints_x, map_waypoints_y, map_waypoints_s,
        future_sensor_fusion,
        possible_trajectories[i]->state_,
        possible_trajectories[i], // this pointer is modified to link to lowest cost next trajectory
        depth
      );
    }
  } else {

    for (int i = 0; i < possible_trajectories.size(); i++) {
      shared_ptr<PossibleTrajectory> current = possible_trajectories[0]; 
      while (current->prev_) {
        if (!current->prev_->lowest_) {
          current->prev_->lowest_ = current;
          // propogate total cost back to initial state
          current->prev_->total_cost_ = current->total_cost_;
        } else if (current->total_cost_ < current->prev_->lowest_->total_cost_) {
          current->prev_->lowest_ = current;
          // propogate total cost back to initial state
          current->prev_->total_cost_ = current->total_cost_;
        }

        current = current->prev_;
      }
    }
  } 

  return possible_trajectories;
}

vector<shared_ptr<PossibleTrajectory>> BehaviorPlanner::getPossibleTrajectories(
  double car_x,double car_y, double car_s, double car_d, double car_yaw,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s,
  vector<vector<double>> sensor_fusion,
  shared_ptr<State> fromState,
  shared_ptr<PossibleTrajectory> prev_possible_trajectory
) {

  Trajectory trajectory;
  vector<shared_ptr<PossibleTrajectory>> trajectories;
  vector<shared_ptr<Cost>> costs = {
    shared_ptr<Cost>(new SlowSpeedCost()),
    shared_ptr<Cost>(new CollideCost()),
    shared_ptr<Cost>(new TooCloseCost()),
    shared_ptr<Cost>(new ChangeLaneCost())
  };

  int car_lane = trajectory.getLaneNumber(car_d);
  int closest_vehicle_id = trajectory.getClosestVehicleId(car_d, car_s, sensor_fusion);
  vector<State::StateId> transitions = fromState->getTransitions(car_lane);

  double closest_vehicle_s = 0;
  double diff_closest_s = 0;
  if (closest_vehicle_id != -1) {
    closest_vehicle_s = sensor_fusion[closest_vehicle_id][5];
    diff_closest_s = trajectory.distance(car_s, closest_vehicle_s);
  }

  // get target vehicle ids
  // also append -1 for no target vehicle
  vector<int> target_vehicle_ids;
  target_vehicle_ids.push_back(-1);
  for (int sf_idx = 0; sf_idx < sensor_fusion.size(); sf_idx++) {
    target_vehicle_ids.push_back(sensor_fusion[sf_idx][0]);
  }

  for (int t_idx = 0; t_idx < transitions.size(); t_idx++) {

    for (int v_idx = 0; v_idx < target_vehicle_ids.size(); v_idx++) {

      // initialize variables to car reference point
      int target_vehicle_lane = car_lane;
      double diff_s = 0;

      // get target vehicle lane and distance
      int target_vehicle_id = target_vehicle_ids[v_idx];
      if (target_vehicle_id != -1) { 
        double target_vehicle_s = sensor_fusion[target_vehicle_id][5];
        double target_vehicle_d = sensor_fusion[target_vehicle_id][6];
        
        diff_s = trajectory.distance(car_s, target_vehicle_s);
        target_vehicle_lane = trajectory.getLaneNumber(target_vehicle_d);
      }

      // find trajectories for each lane
      vector<int> target_lanes = {0, 1, 2};
      for (int l_idx = 0; l_idx < target_lanes.size(); l_idx++) {

        shared_ptr<State> toState = StateFactory::create(
          transitions[t_idx],
          target_vehicle_id,
          target_vehicle_lane,
          target_lanes[l_idx]
        );

        // filter out invalid transitions
        if (!toState->isValid(fromState, car_lane, diff_s, diff_closest_s)) {
          continue;
        };

        // create the trajectory
        vector<vector<double>> possible_trajectory = trajectory.getTrajectory(
          toState,
          sensor_fusion,
          car_x, car_y, car_s, car_d, car_yaw,
          previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s
        );

        // score trajectory using cost functions
        double cost = 0.;
        for (int c_idx = 0; c_idx < costs.size(); c_idx++) {
          double amount = costs[c_idx]->getCost(
            fromState,
            toState,
            car_x,
            car_y,
            possible_trajectory,
            sensor_fusion,
            map_waypoints_x,
            map_waypoints_y
          );
          cost += amount;
        }

        // store trajectory
        trajectories.push_back(shared_ptr<PossibleTrajectory>(
          new PossibleTrajectory(
            toState,
            possible_trajectory,
            cost,
            target_vehicle_id,
            target_lanes[l_idx],
            prev_possible_trajectory
          )
        ));
      }
    }
  }

  return trajectories;
}

vector<vector<double>> BehaviorPlanner::getFutureSensorFusion(
  int N,
  vector<double> maps_x,
  vector<double> maps_y,
  vector<double> maps_s,
  vector<vector<double>> sensor_fusion
) {
  Trajectory trajectory;
  vector<vector<double>> new_sensor_fusion;

  for (int sf_idx = 0; sf_idx < sensor_fusion.size(); sf_idx++) {

    double vx = sensor_fusion[sf_idx][3];
    double vy = sensor_fusion[sf_idx][4];
    double velocity = trajectory.velocity(vx, vy);

    double vehicle_s = sensor_fusion[sf_idx][5] + trajectory.distance(velocity) * N;
    double vehicle_d = sensor_fusion[sf_idx][6];
    vector<double> xy = trajectory.getXY(vehicle_s, vehicle_d, maps_s, maps_x, maps_y);

    new_sensor_fusion.push_back({
      sensor_fusion[sf_idx][0], // id
      xy[0], // x
      xy[1], // y
      vx,
      vy,
      vehicle_s,
      vehicle_d
    });
  }

  return new_sensor_fusion;
}
