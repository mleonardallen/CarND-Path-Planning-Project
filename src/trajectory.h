#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include "util.h"
#include "spline.h"

class Trajectory {
 public:
  Trajectory();

  virtual ~Trajectory();

  std::vector<std::vector<double>> getFutureTrajectory(
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

  std::vector<std::vector<double>> getTrajectory(
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

 private:

  Util util_;

  double ref_vel_ = 49.5;
  int lane_ = 1;
  int num_path_ = 50;
  // each lane is 4 m wide
  double lane_size_ = 4.0;
  double lane_center_offset_ = 2.0;
  // The simulator runs a cycle every 20 ms (50 frames per second), but your C++ path planning program will provide new a new path at least one 20 ms cycle behind
  double simulator_cycle_ = 0.02;
  

};

#endif /* TRAJECTORY_H */