#include <math.h>
#include <iostream>
#include <numeric>
#include "trajectory.h"
#include "behavior.h"
#include "spline.h"

using namespace std;

Trajectory::Trajectory() {
}
Trajectory::~Trajectory() {}

vector<vector<double>> Trajectory::getTrajectory(
  shared_ptr<State> transistion,
  vector<vector<double>> sensor_fusion,
  double car_x,
  double car_y,
  double car_s,
  double car_d,
  double car_yaw,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s
) {

  double target_vehicle_id = transistion->target_vehicle_id_;
  int car_lane = getLaneNumber(car_d);
  if (target_vehicle_id == -1 && car_lane == transistion->target_lane_) {
    double closest_vehicle_id = getClosestVehicleId(car_d, car_s, sensor_fusion);
    target_vehicle_id = closest_vehicle_id;
  }
  double max_vel = max_vel_;
  if (target_vehicle_id != -1) {
    vector<double> target_vehicle = sensor_fusion[target_vehicle_id];
    double target_vehicle_s = target_vehicle[5];
    max_vel = getLeadingVelocity(car_s, target_vehicle);
  }

  // Build trajector from previous points and future points
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // retain previous trajectory points
  next_x_vals.insert(next_x_vals.end(), previous_path_x.begin(), previous_path_x.end());
  next_y_vals.insert(next_y_vals.end(), previous_path_y.begin(), previous_path_y.end());

  // generate new trajectory points
  vector<vector<double>> future = getFutureTrajectory(
    transistion->target_lane_,
    sensor_fusion,
    car_x, car_y, car_s, car_d, car_yaw,
    previous_path_x, previous_path_y,
    map_waypoints_x, map_waypoints_y, map_waypoints_s,
    max_vel
  );
  next_x_vals.insert(next_x_vals.end(), future[0].begin(), future[0].end());
  next_y_vals.insert(next_y_vals.end(), future[1].begin(), future[1].end());

  return {next_x_vals, next_y_vals};
}

vector<vector<double>> Trajectory::getFutureTrajectory(
  int target_lane,
  vector<vector<double>> sensor_fusion,
  double car_x,
  double car_y,
  double car_s,
  double car_d,
  double car_yaw,
  vector<double> previous_path_x,
  vector<double> previous_path_y,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s,
  double max_vel
) {
  std::vector<double> ptsx;
  std::vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);
  int prev_size = previous_path_x.size();

  // get previous points for spline (previous points bring continuity)
  if (prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // get future points for spline
  vector<int> distances = {30, 60, 90};
  for (int i = 0; i < distances.size(); i++) {
    vector<double> xy = getXY(
      car_s + distances[i],
      lane_center_offset_ + lane_size_ * target_lane,
      map_waypoints_s, map_waypoints_x, map_waypoints_y
    );
    ptsx.push_back(xy[0]);
    ptsy.push_back(xy[1]);
  }

  // convert points to local space
  for (int i = 0; i < ptsx.size(); i++) {
    vector<double> xy = getLocalSpace(ptsx[i], ptsy[i], ref_x, ref_y, ref_yaw);
    ptsx[i] = xy[0];
    ptsy[i] = xy[1];
  }

  // create the spline
  tk::spline spline;
  spline.set_points(ptsx, ptsy);

  // convert spline to points
  vector<double> x_vals;
  vector<double> y_vals;

  double target_x = num_path_;
  double target_y = spline(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  // get the previous path velocity
  double ref_vel = previous_path_x.size() 
    ? ref_vel = getAverageVelocity({previous_path_x, previous_path_y})
    : 0;

  // increase/decrease speed to match reference veolocity
  double increment = 5.0;
  if (ref_vel < max_vel) {
    double percent = (1 - (ref_vel / max_vel));
    ref_vel += percent * increment;
  } else {
    ref_vel -= increment;
  }

  double N = target_dist / distance(ref_vel);
  double x_add_on = 0;

  for (int i = 1; i <= num_path_ - prev_size; i++) {

    double x_point = x_add_on + target_x / N;
    double y_point = spline(x_point);

    x_add_on = x_point;

    // rotate back to normal after rotating earlier
    vector<double> xy = getGlobalSpace(x_point, y_point, ref_x, ref_y, ref_yaw);

    x_vals.push_back(xy[0]);
    y_vals.push_back(xy[1]);
  }

  return {x_vals, y_vals};
}

double Trajectory::getLeadingVelocity(double car_s, vector<double> target_vehicle) {

  double target_vehicle_vx = target_vehicle[3];
  double target_vehicle_vy = target_vehicle[4];
  double target_vehicle_s = target_vehicle[5];
  double target_vehicle_d = target_vehicle[6];
  double target_vehicle_speed = velocity(target_vehicle_vx, target_vehicle_vy);

  // ramp down velocity slowly if approaching car
  double diff_s = distance(car_s, target_vehicle_s);
  if (diff_s < num_path_) {
    double percent_ref_ = (num_path_ - diff_s) / num_path_;
    double diff_max = max_vel_ - target_vehicle_speed;
    return max_vel_ - diff_max * percent_ref_;
  }

  return max_vel_;
}

// 
int Trajectory::getClosestVehicleId(
  double car_d, 
  double car_s,
  vector<vector<double>> sensor_fusion
) {
  // get closest vehicle in current lane
  // todo pass in car_lane
  int car_lane = getLaneNumber(car_d);
  int closest_vehicle_id = -1;
  double closest_diff_s = max_s_;

  for (int sf_idx = 0; sf_idx < sensor_fusion.size(); sf_idx++) {

    int target_vehicle_id = sensor_fusion[sf_idx][0];
    double target_vehicle_s = sensor_fusion[sf_idx][5];
    double target_vehicle_d = sensor_fusion[sf_idx][6];

    double target_vehicle_lane = getLaneNumber(target_vehicle_d);
    double diff_s = distance(car_s, target_vehicle_s);

    if (
      target_vehicle_lane != car_lane ||
      diff_s < 0 // already passed car
    ) {
      continue;
    }

    if (diff_s < closest_diff_s) {
      closest_diff_s = diff_s;
      closest_vehicle_id = target_vehicle_id;
    }
  }

  return closest_vehicle_id;
}

double Trajectory::getAverageVelocity(vector<vector<double>> waypoints) {

  double x1 = waypoints[0][0];
  double y1 = waypoints[1][0];

  vector<double> speeds;
  for (int idx = 1; idx < waypoints[0].size(); idx++) {
    double x2 = waypoints[0][idx];
    double y2 = waypoints[1][idx];
    double dist = distance(x1, y1, x2, y2);
    double speed = velocity(dist);
    speeds.push_back(speed);

    x1 = x2;
    y1 = y2;
  }

  double average = accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
  return average;
}

// For converting back and forth between radians and degrees.
double Trajectory::pi() { return M_PI; }
double Trajectory::deg2rad(double x) { return x * pi() / 180; }
double Trajectory::rad2deg(double x) { return x * 180 / pi(); }

double Trajectory::velocity(double vx, double vy) {return sqrt(vx * vx + vy * vy);}
double Trajectory::velocity(double distance) {return distance / cycle_time_ms_;}
double Trajectory::distance(double velocity) {return velocity * cycle_time_ms_;}
double Trajectory::distance(double x1, double y1, double x2, double y2) {return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));}
double Trajectory::distance(double s1, double s2) {
  double largest_s = s1 > s2 ? s1 : s2;
  double shifted_s1 = s1 - largest_s + max_s_;
  double shifted_s2 = s2 - largest_s + max_s_;
  return shifted_s2 - shifted_s1;
}

int Trajectory::getLaneNumber(double d) {
  return (int) floor(d / lane_size_);
}

vector<double> Trajectory::getLocalSpace(
  double x,
  double y,
  double ref_x,
  double ref_y,
  double ref_yaw
) {

  double shift_x = x - ref_x;
  double shift_y = y - ref_y;

  x = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
  y = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);

  return {x, y};
}

vector<double> Trajectory::getGlobalSpace(double x, double y, double ref_x, double ref_y, double ref_yaw) {
  double local_x = x;
  double local_y = y;
  x = local_x * cos(ref_yaw) - local_y * sin(ref_yaw);
  y = local_x * sin(ref_yaw) + local_y * cos(ref_yaw);
  return {x + ref_x, y + ref_y};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Trajectory::getXY(double s, double  d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }  

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Trajectory::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int next_wp = NextWaypoint(x, y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

int Trajectory::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int Trajectory::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = fabs(theta-heading);

  if (angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;
}

double Trajectory::getMaxVelocity() {
  return max_vel_;
}
