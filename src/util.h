#ifndef UTIL_H
#define UTIL_H

#include <vector>

class Util {
 public:
  Util();

  virtual ~Util();

  double pi();

  double deg2rad(double x);
  double rad2deg(double x);
  double distance(double x1, double y1, double x2, double y2);

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

};

#endif /* UTIL_H */