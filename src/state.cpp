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
  cout << name_ << ": " << "Vehcile " << target_vehicle_id_ << ", Vehicle Lane " << target_vehicle_lane_ << ", Target Lane " << target_lane_ << endl;;
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

bool LaneKeepState::isValid(shared_ptr<State> state, int car_lane, double diff_s, double diff_closest_s) {
  return car_lane == target_vehicle_lane_
    && car_lane == target_lane_
    && target_vehicle_id_ == -1;
}

vector<State::StateId> LaneKeepState::getTransitions() {
  return {
    State::StateId::LANE_KEEP,
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

bool PrepareLaneChangeLeftState::isValid(shared_ptr<State> state, int car_lane, double diff_s, double diff_closest_s) {
  // if no leading vehicle, then we assume that we are ready to change lanes,
  // so we should not continue to prepare to change lanes

  if (
    state->getId() == State::StateId::PREPARE_LANE_CHANGE_LEFT
    && target_vehicle_id_ == -1
  ) return false;

  if (
    target_vehicle_id_ != -1
    && diff_closest_s < diff_s
  ) return false;

  return (
    target_vehicle_lane_ >= 0
    && target_vehicle_lane_ <= 2
    && car_lane > 0
    && car_lane == target_lane_ // target lane is current lane
    // distance is in range of car
    && diff_s > - 10
    && diff_s < 60
  );
}

vector<State::StateId> PrepareLaneChangeLeftState::getTransitions() {
  return {
    State::StateId::PREPARE_LANE_CHANGE_LEFT,
    State::StateId::LANE_CHANGE_LEFT,
    State::StateId::LANE_KEEP
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

bool PrepareLaneChangeRightState::isValid(shared_ptr<State> state, int car_lane, double diff_s, double diff_closest_s) {
  // if no leading vehicle, then we assume that we are ready to change lanes,
  // so we should not continue to prepare to change lanes
  if (
    state->getId() == State::StateId::PREPARE_LANE_CHANGE_RIGHT
    && target_vehicle_id_ == -1
  ) return false;

  if (
    target_vehicle_id_ != -1
    && diff_closest_s < diff_s
  ) return false;

  return (
    target_vehicle_lane_ >= 0
    && target_vehicle_lane_ <= 2
    && car_lane < 2
    && car_lane == target_lane_
    // distance is in range of car
    && diff_s > - 10
    && diff_s < 60
  );
}

vector<State::StateId> PrepareLaneChangeRightState::getTransitions() {
  return {
    State::StateId::PREPARE_LANE_CHANGE_RIGHT,
    State::StateId::LANE_CHANGE_RIGHT,
    State::StateId::LANE_KEEP
  };
}

// Lane Change Left

LaneChangeLeftState::LaneChangeLeftState(int target_vehicle_id, int target_vehicle_lane, int target_lane) {
  id_ = State::StateId::LANE_CHANGE_LEFT;
  name_ = "Lane Change Left <=";
  target_vehicle_id_ = target_vehicle_id;
  target_vehicle_lane_ = target_vehicle_lane;
  target_lane_ = target_lane;
}

bool LaneChangeLeftState::isValid(shared_ptr<State> state, int car_lane, double diff_s, double diff_closest_s) {
  return (
    target_vehicle_lane_ > 0
    && target_vehicle_lane_ < 2
    && target_lane_ == car_lane - 1 // target lane is to the left
    // distance is in range of car
    && diff_s > - 10
    && diff_s < 60
  );
}

vector<State::StateId> LaneChangeLeftState::getTransitions() {
  return {
    State::StateId::LANE_CHANGE_LEFT,
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

bool LaneChangeRightState::isValid(shared_ptr<State> state, int car_lane, double diff_s, double diff_closest_s) {
  return (
    target_vehicle_lane_ > 0
    && target_vehicle_lane_ < 2
    && target_lane_ == car_lane + 1 // target lane is to the right
    // distance is in range of car
    && diff_s > - 10
    && diff_s < 60
  );
}

vector<State::StateId> LaneChangeRightState::getTransitions() {
  return {
    State::StateId::LANE_CHANGE_RIGHT,
    State::StateId::PREPARE_LANE_CHANGE_RIGHT,
    State::StateId::LANE_KEEP
  };
}
