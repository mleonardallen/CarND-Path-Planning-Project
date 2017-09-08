#include <iostream>
#include <memory>
#include <iomanip>
#include "behavior.h"
#include "trajectory.h"
#include "state.h"
#include "cost.h"

using namespace std;

PossibleTrajectory::PossibleTrajectory(
    shared_ptr<State> state,
    vector<vector<double>> trajectory,
    vector<vector<double>> sensor_fusion,
    double cost,
    double car_x, double car_y, double car_s, double car_d, double car_yaw,
    int target_leading_vehicle_id,
    int target_lane_id,
    shared_ptr<PossibleTrajectory> prev
) {
  state_ = state;
  trajectory_ = trajectory;
  sensor_fusion_ = sensor_fusion;
  cost_ = cost;

  car_x_ = car_x;
  car_y_ = car_y;
  car_s_ = car_s;
  car_d_ = car_d;
  car_yaw_ = car_yaw;

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

void PossibleTrajectory::print(
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y
) {
  Trajectory trajectory;
  cout << "Name: " << left << setw(30) << state_->name_;
  cout << "Vehcile " << state_->target_vehicle_id_;
  cout << "Vehcile " << target_leading_vehicle_id_;
  cout << "\tVehicle Lane " << state_->target_vehicle_lane_;
  cout << "\tTarget Lane " << state_->target_lane_;
  cout << "\tCost: " << total_cost_;
  cout << "\tAvg V: " << trajectory.getAverageVelocity(trajectory_);
  cout << "\tMax V: " << trajectory.getMaxVelocity(car_s_, car_d_, state_, sensor_fusion_);

  int id = trajectory.getClosestVehicleId(car_d_, car_s_, sensor_fusion_);
  cout << "\tClosest Vehicle: " << id;

  if (id != -1) {
    cout << "\tDistance: " << trajectory.distanceS1S2(car_s_, sensor_fusion_[id][5]);
  }

  vector<double> xs = trajectory_[0];
  vector<double> ys = trajectory_[1];
  double x = xs.back();
  double y = ys.back();
  vector<double> sd = trajectory.getFrenet(x, y, 0, map_waypoints_x, map_waypoints_y);
  cout << "\tS: " << sd[0];
  cout << "\tD: " << sd[1];
  
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
  for (int idx = 0; idx < possible_trajectories.size(); idx++) {
    shared_ptr<PossibleTrajectory> current = possible_trajectories[idx];
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

  Trajectory trajectory;
  vector<shared_ptr<PossibleTrajectory>> possible_trajectories = BehaviorPlanner::getPossibleTrajectories(
    car_x, car_y, car_s, car_d, car_yaw,
    previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
    sensor_fusion,
    state,
    prev
  );

  if (depth > 1) {
    depth = depth - 1;
    for (int i = 0; i < possible_trajectories.size(); i++) {
      // - RECURSION
      previous_path_x = possible_trajectories[i]->trajectory_[0];
      previous_path_y = possible_trajectories[i]->trajectory_[1];
      car_x = previous_path_x.back();
      car_y = previous_path_y.back();
      vector<double> sd = trajectory.getFrenet(car_x, car_y, 0, map_waypoints_x, map_waypoints_y);

      int N = trajectory.num_path_;
      sensor_fusion = trajectory.getFutureSensorFusion(map_waypoints_x, map_waypoints_y, map_waypoints_s, sensor_fusion, N - 2);
      getPossibleTrajectoriesRecursive(
        car_x, car_y, sd[0], sd[1], car_yaw,
        // previous path contains a couple points so we can get velocity
        {previous_path_x[N - 2], previous_path_x[N - 1]},
        {previous_path_y[N - 2], previous_path_y[N - 1]},
        map_waypoints_x, map_waypoints_y, map_waypoints_s,
        sensor_fusion,
        possible_trajectories[i]->state_,
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
  shared_ptr<State> fromState,
  shared_ptr<PossibleTrajectory> prev_possible_trajectory
) {

  Trajectory trajectory;
  vector<shared_ptr<PossibleTrajectory>> trajectories;

  // cost functions
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
    diff_closest_s = trajectory.distanceS1S2(car_s, closest_vehicle_s);
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
        
        diff_s = trajectory.distanceS1S2(car_s, target_vehicle_s);
        target_vehicle_lane = trajectory.getLaneNumber(target_vehicle_d);
      }

      // ignore vehicles that aren't on this side of the road
      if (!(target_vehicle_lane >= 0 && target_vehicle_lane <= 2)) {
        continue;
      };

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
            sensor_fusion,
            cost,
            car_x, car_y, car_s, car_d, car_yaw,
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


