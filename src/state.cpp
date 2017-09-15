#include <iostream>
#include <memory>
#include <string>
#include "trajectory.h"
#include "state.h"

using namespace std;

State::State() {}
State::~State() {}

State::StateId State::getId() {
  return id_;
}

void State::print() {
  cout << name_ << ": " << "Vehcile " << target_vehicle_id_ << ", Vehicle Lane " << target_vehicle_lane_ << ", Target Lane " << target_lane_ << endl;
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
  int target_vehicle_id,
  int target_vehicle_lane,
  int target_lane
) {

  if (id == State::StateId::PREPARE_LANE_CHANGE_LEFT) {
    return shared_ptr<State>(new PrepareLaneChangeLeftState(target_vehicle_id, target_vehicle_lane, target_lane));
  }

  if (id == State::StateId::PREPARE_LANE_CHANGE_RIGHT) {
    return shared_ptr<State>(new PrepareLaneChangeRightState(target_vehicle_id, target_vehicle_lane, target_lane));
  }

  if (id == State::StateId::LANE_CHANGE_LEFT) {
    return shared_ptr<State>(new LaneChangeLeftState(target_vehicle_id, target_vehicle_lane, target_lane));
  }

  if (id == State::StateId::LANE_CHANGE_RIGHT) {
    return shared_ptr<State>(new LaneChangeRightState(target_vehicle_id, target_vehicle_lane, target_lane));
  }

  // default state is lane keep state
  return shared_ptr<State>(new LaneKeepState(target_vehicle_id, target_vehicle_lane, target_lane));
}

// Lane Keep

LaneKeepState::LaneKeepState(int target_vehicle_id, int target_vehicle_lane, int target_lane) {
  id_ = State::StateId::LANE_KEEP;
  name_ = "Lane Keep";
  target_vehicle_id_ = target_vehicle_id;
  target_vehicle_lane_ = target_vehicle_lane;
  target_lane_ = target_lane;
}

bool LaneKeepState::isValid(shared_ptr<State> fromState, int car_lane, double diff_s, double diff_closest_s) {
  return car_lane == target_vehicle_lane_
    && car_lane == target_lane_
    && target_vehicle_id_ == -1;
}

bool LaneKeepState::isComplete(
  double car_x,
  double car_y,
  double car_s,
  double car_d,
  double car_yaw,
  vector<vector<double>> sensor_fusion
) {
  return true;
}

vector<State::StateId> LaneKeepState::getTransitions() {
  return {
    State::StateId::LANE_KEEP
    ,
    State::StateId::PREPARE_LANE_CHANGE_LEFT,
    State::StateId::PREPARE_LANE_CHANGE_RIGHT
  };
}

// Prepare Lane Change Left

PrepareLaneChangeLeftState::PrepareLaneChangeLeftState(int target_vehicle_id, int target_vehicle_lane, int target_lane) {
  id_ = State::StateId::PREPARE_LANE_CHANGE_LEFT;
  name_ = "Prepare Lane Change Left <-";
  target_vehicle_id_ = target_vehicle_id;
  target_vehicle_lane_ = target_vehicle_lane;
  target_lane_ = target_lane;
}

bool PrepareLaneChangeLeftState::isValid(shared_ptr<State> fromState, int car_lane, double diff_s, double diff_closest_s) {

  // if no leading vehicle, then we assume that we are ready to change lanes,
  // so we should not continue to prepare to change lanes
  if (
    fromState->getId() == State::StateId::PREPARE_LANE_CHANGE_LEFT
    && target_vehicle_id_ == -1
  ) return false;

  // do not follow car if the car in front of you is closer.
  if (
    target_vehicle_id_ != -1
    && diff_closest_s < diff_s
  ) return false;

  // don't use as target vehicle if it's not in the left lane
  if (
    target_vehicle_id_ != -1
    && target_vehicle_lane_ != car_lane - 1
  ) return false;

  return (
    car_lane > 0
    && car_lane == target_lane_ // target lane is current lane because we aren't turning yet
    // distance is in range of car
    && diff_s > lower_limit_
    && diff_s < upper_limit_
  );
}

bool PrepareLaneChangeLeftState::isComplete(
  double car_x,
  double car_y,
  double car_s,
  double car_d,
  double car_yaw,
  vector<vector<double>> sensor_fusion
) {
  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);
  // make sure left lane is clear
  bool is_clear = true;
  for (int sf_idx = 0; sf_idx < sensor_fusion.size(); sf_idx++) {

    double s = sensor_fusion[sf_idx][5];
    double d = sensor_fusion[sf_idx][6];

    // only look at cars in the left lane
    double lane = trajectory.getLaneNumber(d);
    if (lane != car_lane - 1) continue;

    double distance = trajectory.distanceS1S2(car_s, s);
    if (distance < 14 && distance > - 8) {
      is_clear = false;
    }
  }

  return is_clear;
}

vector<State::StateId> PrepareLaneChangeLeftState::getTransitions() {
  return {
    State::StateId::LANE_CHANGE_LEFT
  };
}

// Prepare Lane Change Right

PrepareLaneChangeRightState::PrepareLaneChangeRightState(int target_vehicle_id, int target_vehicle_lane, int target_lane) {

  id_ = State::StateId::PREPARE_LANE_CHANGE_RIGHT;
  name_ = "Prepare Lane Change Right ->";

  target_vehicle_id_ = target_vehicle_id;
  target_vehicle_lane_ = target_vehicle_lane;
  target_lane_ = target_lane;
}

bool PrepareLaneChangeRightState::isValid(shared_ptr<State> fromState, int car_lane, double diff_s, double diff_closest_s) {
  // if no leading vehicle, then we assume that we are ready to change lanes,
  // so we should not continue to prepare to change lanes
  if (
    fromState->getId() == State::StateId::PREPARE_LANE_CHANGE_RIGHT
    && target_vehicle_id_ == -1
  ) return false;

  if (
    target_vehicle_id_ != -1
    && diff_closest_s < diff_s
  ) return false;

  // don't use as target vehicle if it's not in the right lane
  if (
    target_vehicle_id_ != -1
    && target_vehicle_lane_ != car_lane + 1
  ) return false;

  return (
    car_lane < 2
    && car_lane == target_lane_
    // distance is in range of car
    && diff_s > lower_limit_
    && diff_s < upper_limit_
  );
}

vector<State::StateId> PrepareLaneChangeRightState::getTransitions() {
  return {
    State::StateId::LANE_CHANGE_RIGHT
  };
}

bool PrepareLaneChangeRightState::isComplete(
  double car_x,
  double car_y,
  double car_s,
  double car_d,
  double car_yaw,
  vector<vector<double>> sensor_fusion
) {
  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);
  // make sure left lane is clear
  bool is_clear = true;
  for (int sf_idx = 0; sf_idx < sensor_fusion.size(); sf_idx++) {

    double s = sensor_fusion[sf_idx][5];
    double d = sensor_fusion[sf_idx][6];

    // only look at cars in the left lane
    double lane = trajectory.getLaneNumber(d);
    if (lane != car_lane + 1) continue;

    double distance = trajectory.distanceS1S2(car_s, s);
    if (distance < 14 && distance > - 8) {
      is_clear = false;
    }
  }

  return is_clear;
}

// Lane Change Left

LaneChangeLeftState::LaneChangeLeftState(int target_vehicle_id, int target_vehicle_lane, int target_lane) {
  id_ = State::StateId::LANE_CHANGE_LEFT;
  name_ = "Lane Change Left <=";
  target_vehicle_id_ = target_vehicle_id;
  target_vehicle_lane_ = target_vehicle_lane;
  target_lane_ = target_lane;
}

bool LaneChangeLeftState::isValid(shared_ptr<State> fromState, int car_lane, double diff_s, double diff_closest_s) {
  // if target vehicle is not in target lane, it is not valid
  if (
    target_vehicle_id_ != -1
    && target_vehicle_lane_ != target_lane_
  ) {
    return false;
  }

  return (
    target_lane_ == car_lane - 1 // target lane is to the left
    // distance is in range of car
    && diff_s > lower_limit_
    && diff_s < upper_limit_
  );
}

bool LaneChangeLeftState::isComplete(
  double car_x,
  double car_y,
  double car_s,
  double car_d,
  double car_yaw,
  vector<vector<double>> sensor_fusion
) {
  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);
  return target_lane_ == car_lane;
}

vector<State::StateId> LaneChangeLeftState::getTransitions() {
  return {
    State::StateId::PREPARE_LANE_CHANGE_LEFT,
    State::StateId::LANE_KEEP
  };
}

// Lane Change Right

LaneChangeRightState::LaneChangeRightState(int target_vehicle_id, int target_vehicle_lane, int target_lane) {
  id_ = State::StateId::LANE_CHANGE_RIGHT;
  name_ = "Lane Change Right =>";
  target_vehicle_id_ = target_vehicle_id;
  target_vehicle_lane_ = target_vehicle_lane;
  target_lane_ = target_lane;
}

bool LaneChangeRightState::isValid(shared_ptr<State> fromState, int car_lane, double diff_s, double diff_closest_s) {
  // if target vehicle is not in target lane, it is not valid
  if (
    target_vehicle_id_ != -1
    && target_vehicle_lane_ != target_lane_
  ) {
    return false;
  }

  return (
    target_lane_ == car_lane + 1 // target lane is to the right
    // distance is in range of car
    && diff_s > lower_limit_
    && diff_s < upper_limit_
  );
}

bool LaneChangeRightState::isComplete(
  double car_x,
  double car_y,
  double car_s,
  double car_d,
  double car_yaw,
  vector<vector<double>> sensor_fusion
) {
  Trajectory trajectory;
  int car_lane = trajectory.getLaneNumber(car_d);
  return target_lane_ == car_lane;
}

vector<State::StateId> LaneChangeRightState::getTransitions() {
  return {
    State::StateId::PREPARE_LANE_CHANGE_RIGHT,
    State::StateId::LANE_KEEP
  };
}
