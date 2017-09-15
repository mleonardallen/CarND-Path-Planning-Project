#ifndef PRECICTION_H
#define PRECICTION_H

#include <vector>
#include <map>

class Prediction {
 public:
  Prediction(
    double vehicle_id,
    double average_velocity,
    double average_acceleration
  );
  virtual ~Prediction();
  void print();

  double vehicle_id_;
  double average_velocity_;
  double average_acceleration_;
};

class Predictor {
 public:
  Predictor();
  virtual ~Predictor();

  std::map<int, std::shared_ptr<Prediction>> getPredictions(std::vector<std::vector<std::vector<double>>> sensor_fusion_history);

  std::vector<std::vector<double>> getFutureSensorFusion(
    std::vector<double> maps_x,
    std::vector<double> maps_y,
    std::vector<double> maps_s,
    std::vector<std::vector<double>> sensor_fusion,
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    int N
  );

  std::vector<std::vector<std::vector<double>>> updateHistory(
    std::vector<std::vector<std::vector<double>>> sensor_fusion_history,
    std::vector<std::vector<double>> sensor_fusion
  );
};

#endif /* PRECICTION_H */