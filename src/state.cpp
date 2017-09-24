#include <iostream>
#include <memory>
#include <string>
#include "trajectory.h"
#include "state.h"
#include "cost.h"
#include <math.h>

using namespace std;

State::State() {}
State::~State() {}

State::StateId State::getId() {
  return id_;
}

string State::getName() {
  return name_;
}

/**
 * Get reference to concrete state
 *
 * @param {State*} pointer to populate with state reference
 * @param {State::StateId} id
 * @return {State*} reference to concrete state
 */
shared_ptr<State> StateFactory::create(
  State::StateId id,
  int target_lane
) {

  if (id == State::StateId::PREPARE_LANE_CHANGE_LEFT) {
    return shared_ptr<State>(new PrepareLaneChangeLeftState(target_lane));
  }

  if (id == State::StateId::PREPARE_LANE_CHANGE_RIGHT) {
    return shared_ptr<State>(new PrepareLaneChangeRightState(target_lane));
  }

  if (id == State::StateId::LANE_CHANGE_LEFT) {
    return shared_ptr<State>(new LaneChangeLeftState(target_lane));
  }

  if (id == State::StateId::LANE_CHANGE_RIGHT) {
    return shared_ptr<State>(new LaneChangeRightState(target_lane));
  }

  // default state is lane keep state
  return shared_ptr<State>(new LaneKeepState(target_lane));
}

// Lane Keep

LaneKeepState::LaneKeepState(int target_lane) {
  id_ = State::StateId::LANE_KEEP;
  name_ = "Lane Keep";
  target_lane_ = target_lane;
}

bool LaneKeepState::isValid(
  shared_ptr<State> fromState,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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

  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);
  return car_lane == target_lane_;
}

bool LaneKeepState::isComplete(
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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
  return true;
}

vector<State::StateId> LaneKeepState::getTransitions() {
  return {
    State::StateId::LANE_KEEP,
    State::StateId::PREPARE_LANE_CHANGE_LEFT,
    State::StateId::PREPARE_LANE_CHANGE_RIGHT
  };
}

// Prepare Lane Change Left

PrepareLaneChangeLeftState::PrepareLaneChangeLeftState(int target_lane) {
  id_ = State::StateId::PREPARE_LANE_CHANGE_LEFT;
  name_ = "Prepare Lane Change Left <-";
  target_lane_ = target_lane;
}

bool PrepareLaneChangeLeftState::isValid(
  shared_ptr<State> fromState,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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

  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);

  return (
    car_lane > 0
    && car_lane == target_lane_ // target lane is current lane because we aren't turning yet
  );
}

bool PrepareLaneChangeLeftState::isComplete(
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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
  Trajectory trajectory;

  // change lane state to generate pretend trajectory
  int target_lane = trajectory.getLaneNumber(car_d) - 1;
  shared_ptr<State> toState = StateFactory::create(State::StateId::LANE_CHANGE_LEFT, target_lane);

  // generate a pretend trajectory to see if we would collide
  int num_waypoints = 75;
  vector<vector<double>> lane_change_waypoints = trajectory.getTrajectory(
    toState,
    sensor_fusion,
    sensor_fusion_history,
    car_x, car_y, car_s, car_d, car_yaw,
    previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
    num_waypoints
  );

  CollideCost cost;
  bool is_collision = cost.isCollision(
    lane_change_waypoints, sensor_fusion, sensor_fusion_history,
    map_waypoints_x, map_waypoints_y, map_waypoints_s,
    true
  );
  return !is_collision;
}

vector<State::StateId> PrepareLaneChangeLeftState::getTransitions() {
  return {
    State::StateId::LANE_KEEP,
    State::StateId::PREPARE_LANE_CHANGE_LEFT,
    State::StateId::LANE_CHANGE_LEFT
  };
}

// Prepare Lane Change Right

PrepareLaneChangeRightState::PrepareLaneChangeRightState(int target_lane) {
  id_ = State::StateId::PREPARE_LANE_CHANGE_RIGHT;
  name_ = "Prepare Lane Change Right ->";
  target_lane_ = target_lane;
}

bool PrepareLaneChangeRightState::isValid(
  shared_ptr<State> fromState,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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

  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);

  return (
    car_lane < 2
    && car_lane == target_lane_
  );
}

vector<State::StateId> PrepareLaneChangeRightState::getTransitions() {
  return {
    State::StateId::LANE_KEEP,
    State::StateId::PREPARE_LANE_CHANGE_RIGHT,
    State::StateId::LANE_CHANGE_RIGHT
  };
}

bool PrepareLaneChangeRightState::isComplete(
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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
  Trajectory trajectory;

  // change lane state to generate pretend trajectory
  int target_lane = trajectory.getLaneNumber(car_d) + 1;
  shared_ptr<State> toState = StateFactory::create(State::StateId::LANE_CHANGE_LEFT, target_lane);

  // generate a pretend trajectory to see if we would collide
  int num_waypoints = 75;
  vector<vector<double>> lane_change_waypoints = trajectory.getTrajectory(
    toState,
    sensor_fusion,
    sensor_fusion_history,
    car_x, car_y, car_s, car_d, car_yaw,
    previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s,
    num_waypoints
  );

  CollideCost cost;
  bool is_collision = cost.isCollision(
    lane_change_waypoints, sensor_fusion, sensor_fusion_history,
    map_waypoints_x, map_waypoints_y, map_waypoints_s,
    true
  );
  return !is_collision;
}

// Lane Change Left

LaneChangeLeftState::LaneChangeLeftState(int target_lane) {
  id_ = State::StateId::LANE_CHANGE_LEFT;
  name_ = "Lane Change Left <=";
  target_lane_ = target_lane;
}

bool LaneChangeLeftState::isValid(
  shared_ptr<State> fromState,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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

  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);

  if (
    fromState->id_ == State::StateId::LANE_CHANGE_LEFT
    && fromState->isComplete(
      sensor_fusion, sensor_fusion_history,
      car_x, car_y, car_s, car_d, car_yaw,
      previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s
    )
  ) return false;

  return (
    target_lane_ == car_lane - 1 // target lane is to the left
  );
}

bool LaneChangeLeftState::isComplete(
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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
  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);
  return target_lane_ == car_lane;
}

vector<State::StateId> LaneChangeLeftState::getTransitions() {
  return {
    State::StateId::PREPARE_LANE_CHANGE_LEFT,
    State::StateId::LANE_CHANGE_LEFT,
    State::StateId::LANE_KEEP
  };
}

// Lane Change Right

LaneChangeRightState::LaneChangeRightState(int target_lane) {
  id_ = State::StateId::LANE_CHANGE_RIGHT;
  name_ = "Lane Change Right =>";
  target_lane_ = target_lane;
}

bool LaneChangeRightState::isValid(
  shared_ptr<State> fromState,
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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

  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);

  if (
    fromState->id_ == State::StateId::LANE_CHANGE_RIGHT
    && fromState->isComplete(
      sensor_fusion, sensor_fusion_history,
      car_x, car_y, car_s, car_d, car_yaw,
      previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s
    )
  ) return false;

  return (
    target_lane_ == car_lane + 1 // target lane is to the right
  );
}

bool LaneChangeRightState::isComplete(
  vector<vector<double>> sensor_fusion,
  vector<vector<vector<double>>> sensor_fusion_history,
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
  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);
  return target_lane_ == car_lane;
}

vector<State::StateId> LaneChangeRightState::getTransitions() {
  return {
    State::StateId::PREPARE_LANE_CHANGE_RIGHT,
    State::StateId::LANE_CHANGE_RIGHT,
    State::StateId::LANE_KEEP
  };
}
