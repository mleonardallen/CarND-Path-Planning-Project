#ifndef STATE_H
#define STATE_H

#include <vector>

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

  State(StateId id);
  virtual ~State();

  void addTransition(State* state);
  void setId(StateId id);
  StateId getId();
  std::vector<State*> getTransitions();

 private:
  std::vector<State*> transition_states_;
};


#endif /* STATE_H */