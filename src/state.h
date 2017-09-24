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

  virtual bool isValid(
    std::shared_ptr<State> fromState,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  ) = 0;
  virtual bool isComplete(
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  ) = 0;
  virtual std::vector<State::StateId> getTransitions() = 0;

  std::string name_;
  int target_lane_;
};

class LaneKeepState : public State {
 public:
  LaneKeepState(int target_lane);
  bool isValid(
    std::shared_ptr<State> fromState,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  );
  bool isComplete(
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  );
  std::vector<StateId> getTransitions();
};

class LaneChangeLeftState : public State {
 public:
  LaneChangeLeftState(int target_lane);
  bool isValid(
    std::shared_ptr<State> fromState,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  );
  bool isComplete(
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  );
  std::vector<StateId> getTransitions();
};

class LaneChangeRightState : public State {
 public:
  LaneChangeRightState(int target_lane);
  bool isValid(
    std::shared_ptr<State> fromState,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  );
  bool isComplete(
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  );
  std::vector<StateId> getTransitions();
};

class PrepareLaneChangeLeftState : public State {
 public:
  PrepareLaneChangeLeftState(int target_lane);
  bool isValid(
    std::shared_ptr<State> fromState,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  );
  bool isComplete(
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  );
  std::vector<StateId> getTransitions();
};

class PrepareLaneChangeRightState : public State {
 public:
  PrepareLaneChangeRightState(int target_lane);
  bool isValid(
    std::shared_ptr<State> fromState,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  );
  bool isComplete(
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    double car_x,
    double car_y,
    double car_s,
    double car_d,
    double car_yaw,
    std::vector<double> previous_path_x,
    std::vector<double> previous_path_y,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s
  );
  std::vector<StateId> getTransitions();
};

class StateFactory {
 public:
  static std::shared_ptr<State> create(State::StateId id, int target_lane);
};

#endif /* STATE_H */