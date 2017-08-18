#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "state.h"

class Behavior {
 public:
  Behavior();
  Behavior(const Behavior &);
  virtual ~Behavior();

  void transition();

 private:
  State* state_;
  State ready_;
  State keep_lane_;
  State lane_change_left_;
  State lane_change_right_;
  State prepare_lane_change_left_;
  State prepare_lane_change_right_;

};

#endif /* BEHAVIOR_H */