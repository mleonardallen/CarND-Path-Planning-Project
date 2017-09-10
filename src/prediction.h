#ifndef PRECICTION_H
#define PRECICTION_H

#include <vector>

class Prediction {
 public:
  Prediction(double acceleration);
  virtual ~Prediction();

  double acceleration_;
};

class Predictor {
 public:
  Predictor();
  virtual ~Predictor();

  std::vector<std::shared_ptr<Prediction>> getPredictions(std::vector<std::vector<std::vector<double>>> sensor_fusion_history);

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