#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include "behavior.h"
#include "spline.h"

class Trajectory {
 public:
  Trajectory();

  virtual ~Trajectory();

  std::vector<std::vector<double>> getFutureTrajectory(
    int target_lane_id,
    std::vector<std::vector<double>> sensor_fusion,
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
    std::shared_ptr<State> transition,
    std::vector<std::vector<double>> sensor_fusion,
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

  double getLeadingVelocity(double car_s, std::vector<double> target_vehicle);

  int getLaneNumber(double d);

  int getClosestVehicleId(double car_d, double car_s, std::vector<std::vector<double>> sensor_fusion);

  double pi();

  double deg2rad(double x);
  double rad2deg(double x);

  double velocity(double distance);
  double velocity(double vx, double vy);

  double distance(double velocity);
  double distance(double x1, double y1, double x2, double y2);
  double distance(double s1, double s2);

  std::vector<double> getGlobalSpace(double x, double y, double ref_x, double ref_y, double ref_yaw);

  std::vector<double> getLocalSpace(
    double x,
    double y,
    double ref_x,
    double ref_y,
    double ref_yaw
  );

  std::vector<double> getXY(
    double s,
    double d,
    std::vector<double> map_waypoints_s,
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y
  );

  std::vector<double> getFrenet(
    double x,
    double y,
    double theta, 
    std::vector<double> maps_x,
    std::vector<double> maps_y
  );

  int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);
  int ClosestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y);

 private:

  static double ref_vel_;

  double max_vel_ = 49.5;
  int lane_ = 1;

  // number of waypoints to include in a trajectory
  int num_path_ = 20;

  // each lane is 4 m wide
  double lane_size_ = 4.0;
  double lane_center_offset_ = 2.0;

  // The simulator runs a cycle every 20 ms (50 frames per second),
  // but your C++ path planning program will provide new a new path at least one 20 ms cycle behind
  double cycle_time_ = 0.02;
  double ms_conversion_ = 2.24;
  double cycle_time_ms_ = cycle_time_ / ms_conversion_;

  double max_s_ = 6945.554;
  

};

#endif /* TRAJECTORY_H */