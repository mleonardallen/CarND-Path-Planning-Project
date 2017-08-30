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
    double car_x,
    double car_y,
    std::vector<std::vector<double>> waypoints,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x,
    std::vector<double> maps_y
  );

  virtual double getCost(
    std::shared_ptr<State> state,
    double car_x,
    double car_y,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x,
    std::vector<double> maps_y
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
    std::shared_ptr<State> state,
    double car_x,
    double car_y,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x,
    std::vector<double> maps_y
  );
};

class TooCloseCost : public Cost {
 public:

  TooCloseCost();

  double getCost(
    std::shared_ptr<State> state,
    double car_x,
    double car_y,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x,
    std::vector<double> maps_y
  );
};


class CollideCost : public Cost {
 public:
  CollideCost();

  double getCost(
    std::shared_ptr<State> state,
    double car_x,
    double car_y,
    std::vector<std::vector<double>> trajectory,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<double> maps_x,
    std::vector<double> maps_y
  );
};

#endif /* COST_H */