#ifndef COST_H
#define COST_H

#include <vector>
#include <memory>
#include "trajectory.h"
#include "state.h"

class Cost {
 public:
  Cost();
  virtual ~Cost();

  double getBufferViolations(
    double buffer_s,
    double buffer_d,
    std::vector<std::vector<double>> waypoints,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x, std::vector<double> maps_y, std::vector<double> maps_s
  );

  virtual double getCost(
    std::shared_ptr<State> toState,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x, std::vector<double> maps_y, std::vector<double> maps_s
  ) = 0;

  double weight_;
};

class SpeedLimitCost : public Cost {
 public:
  SpeedLimitCost();
};

class SlowSpeedCost : public Cost {
 public:

  SlowSpeedCost();

  double getCost(
    std::shared_ptr<State> toState,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x, std::vector<double> maps_y, std::vector<double> maps_s
  );
};

class TooCloseCost : public Cost {
 public:

  TooCloseCost();

  double getCost(
    std::shared_ptr<State> toState,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x, std::vector<double> maps_y, std::vector<double> maps_s
  );
};

class CollideCost : public Cost {
 public:
  CollideCost();

  double getCost(
    std::shared_ptr<State> toState,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x, std::vector<double> maps_y, std::vector<double> maps_s
  );

  bool isCollision(
    double buffer_s, double buffer_d,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x, std::vector<double> maps_y, std::vector<double> maps_s
  );
};

class ChangeLaneCost : public Cost {
 public:
  ChangeLaneCost();

  double getCost(
    std::shared_ptr<State> toState,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x, std::vector<double> maps_y, std::vector<double> maps_s
  );
};

#endif /* COST_H */