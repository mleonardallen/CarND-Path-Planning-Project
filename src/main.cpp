#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "trajectory.h"
#include "behavior.h"
#include "state.h"
#include "prediction.h"
#include "cost.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  vector<thread> threads;
  Trajectory trajectory;
  BehaviorPlanner planner;
  Predictor predictor;

  shared_ptr<State> state = StateFactory::create(State::StateId::LANE_KEEP, 1 /*target lane*/);
  vector<shared_ptr<State>> states;
  vector<vector<vector<double>>> sensor_fusion_history;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  bool thread_is_done = false;
  h.onMessage([
    &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
    &trajectory, &planner, &threads, &states, &state, &predictor, &sensor_fusion_history, &thread_is_done
  ](
    uWS::WebSocket<uWS::SERVER> ws,
    char *data, size_t length,
    uWS::OpCode opCode
  ) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          // Filter out cars that are not relavent
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;

          sensor_fusion_history = predictor.updateHistory(sensor_fusion_history, sensor_fusion);

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          // if planner is not currently working, start a new thread
          if (!threads.size() && !states.size())
          {
            vector<vector<double>> planner_waypoints;
            threads.emplace_back(
              // method
              planner.transition,
              // parameters
              car_x, car_y, car_s, car_d, car_yaw,
              previous_path_x, previous_path_y,
              map_waypoints_x, map_waypoints_y, map_waypoints_s,
              sensor_fusion,
              sensor_fusion_history,
              std::ref(states),
              std::ref(state),
              std::ref(thread_is_done)
            );
          } 
          // if planner is finished, leverage instructions
          else if (thread_is_done && threads[0].joinable())
          {
            // received new instructions from the planner
            threads[0].join();
            threads.pop_back();
            thread_is_done = false;
          }

          if (states.size()) {
            state = states[0];
          } else {
            int target_lane = trajectory.getLaneNumber(car_d);
            state = StateFactory::create(State::StateId::LANE_KEEP, target_lane); 
          }

          vector<vector<double>> waypoints = trajectory.getTrajectory(
            state,
            sensor_fusion,
            sensor_fusion_history,
            car_x, car_y, car_s, car_d, car_yaw,
            previous_path_x, previous_path_y,
            map_waypoints_x, map_waypoints_y, map_waypoints_s,
            trajectory.num_path_
          );

          // check to see if we are done with current operation
          // especially important for prepare lane change
          if (
            states.size() 
            && state->isComplete(
              sensor_fusion, sensor_fusion_history,
              car_x, car_y, car_s, car_d, car_yaw,
              previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s
            )
          ) {
            states.erase(states.begin());
          }

          // END

          msgJson["next_x"] = waypoints[0];
          msgJson["next_y"] = waypoints[1];

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
