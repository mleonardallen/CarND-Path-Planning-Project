#ifndef STATE_H
#define STATE_H

#include <vector>
#include <memory>
#include <string>

class State {
 public:

  enum StateId {
    READY,
    LANE_KEEP,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT,
    PREPARE_LANE_CHANGE_LEFT,
    PREPARE_LANE_CHANGE_RIGHT
  } id_;

  State();
  virtual ~State();

  StateId getId();
  std::string getName();
  void print();

  virtual bool isValid(std::shared_ptr<State> state, int car_lane, double diff_s) = 0;
  virtual std::vector<State::StateId> getTransitions() = 0;

  std::string name_;
  int target_vehicle_id_;
  int target_vehicle_lane_;
  int target_lane_;
};

class LaneKeepState : public State {
 public:
  LaneKeepState(int target_vehicle_id, int target_vehicle_lane, int target_lane);
  bool isValid(std::shared_ptr<State> state, int car_lane, double diff_s);
  std::vector<StateId> getTransitions();
};

class LaneChangeLeftState : public State {
 public:
  LaneChangeLeftState(int target_vehicle_id, int target_vehicle_lane, int target_lane);
  bool isValid(std::shared_ptr<State> state, int car_lane, double diff_s);
  std::vector<StateId> getTransitions();
};

class LaneChangeRightState : public State {
 public:
  LaneChangeRightState(int target_vehicle_id, int target_vehicle_lane, int target_lane);
  bool isValid(std::shared_ptr<State> state, int car_lane, double diff_s);
  std::vector<StateId> getTransitions();
};

class PrepareLaneChangeLeftState : public State {
 public:
  PrepareLaneChangeLeftState(int target_vehicle_id, int target_vehicle_lane, int target_lane);
  bool isValid(std::shared_ptr<State> state, int car_lane, double diff_s);
  std::vector<StateId> getTransitions();
};

class PrepareLaneChangeRightState : public State {
 public:
  PrepareLaneChangeRightState(int target_vehicle_id, int target_vehicle_lane, int target_lane);
  bool isValid(std::shared_ptr<State> state, int car_lane, double diff_s);
  std::vector<StateId> getTransitions();
};

class StateFactory {
 public:
  static std::shared_ptr<State> create(State::StateId id, int target_vehicle_id, int target_vehicle_lane, int target_lane);
};

#endif /* STATE_H */