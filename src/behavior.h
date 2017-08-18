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

};

#endif /* BEHAVIOR_H */