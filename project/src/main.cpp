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
#include "spline.h"
#include "vehicle.h"
#include "helper.h"
#include "path.h"

using namespace std;

// for convenience
using json = nlohmann::json;

void calculate_trajectory(Vehicle my_car, int prev_size,
                          vector<double> &previous_path_x, vector<double> &previous_path_y,
                          double lane, double &ref_vel, double &speed_diff,
                          vector<double> &next_x_vals, vector<double> &next_y_vals,
                          Path &my_path){
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = my_car.car_x;
  double ref_y = my_car.car_y;
  // Car yaw comes in deg, convert it to rads.
  double ref_yaw = deg2rad(my_car.car_yaw);

  // If there are at least two points
  if ( prev_size < 2 ) {
    double prev_car_x = my_car.car_x - cos(my_car.car_yaw);
    double prev_car_y = my_car.car_y - sin(my_car.car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(my_car.car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(my_car.car_y);
  } else {
    // Else use the last two points
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // Setting up target points in the future.
  vector<double> next_wp0 = getXY(my_car.car_s + 30, 2 + 4*lane, my_path.map_waypoints_s, my_path.map_waypoints_x, my_path.map_waypoints_y);
  vector<double> next_wp1 = getXY(my_car.car_s + 60, 2 + 4*lane, my_path.map_waypoints_s, my_path.map_waypoints_x, my_path.map_waypoints_y);
  vector<double> next_wp2 = getXY(my_car.car_s + 90, 2 + 4*lane, my_path.map_waypoints_s, my_path.map_waypoints_x, my_path.map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // Making coordinates to local car coordinates.
  for ( int i = 0; i < ptsx.size(); i++ ) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  // Create the spline.
  tk::spline s;
  s.set_points(ptsx, ptsy);

  // Output path points from previous path for continuity.
  for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate distance y position on 30 m ahead.
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for( int i = 1; i < 50 - prev_size; i++ ) {
    ref_vel += speed_diff;
    if ( ref_vel > MAX_SPEED ) {
      ref_vel = MAX_SPEED;
    } else if ( ref_vel < MAX_ACC ) {
      ref_vel = MAX_ACC;
    }
    double N = target_dist / (0.02 * ref_vel / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

void calculate_prediction(vector<vector<double>> &sensor_fusion, Vehicle &my_car, double lane, int prev_size){
  // Prediction : Analysing other cars positions.
  for ( int i = 0; i < sensor_fusion.size(); i++ ) {
    float d = sensor_fusion[i][6];
    int car_lane = -1;
    // is it on the same lane as we are
    if ( d > 0 && d < 4 ) {
      car_lane = 0;
    } else if ( d > 4 && d < 8 ) {
      car_lane = 1;
    } else if ( d > 8 && d < 12 ) {
      car_lane = 2;
    }
    if (car_lane < 0) {
      continue;
    }
    // Find car speed.
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx * vx + vy * vy);
    double check_car_s = sensor_fusion[i][5];
    // Estimate car s position after executing previous trajectory.
    check_car_s += ((double)prev_size * 0.02 * check_speed);

    if ( car_lane == lane ) {
      // Car in our lane.
      my_car.in_lane |= (check_car_s > my_car.car_s) && ((check_car_s - my_car.car_s) < 30);
    } else if ( (car_lane - lane) == -1 ) {
      // Car left
      my_car.car_left |= (my_car.car_s - 30 < check_car_s) && (my_car.car_s + 30 > check_car_s);
    } else if ( (car_lane - lane) == 1 ) {
      // Car right
      my_car.car_right |= ((my_car.car_s - 30) < check_car_s) && ((my_car.car_s + 30) > check_car_s);
    }
  }
}

void plan_behaviour(Vehicle &my_car, int &lane, double &speed_diff, double ref_vel){
  // Behavior : Let's see what to do.
  // Car ahead or in the back
  if (my_car.in_lane) {
    if (!my_car.car_left && lane > 0) {
      // if there is no car left and there is a left lane.
      cout << "| Change lane to the LEFT" << endl;
      lane--; // Change lane left.
    } else if ( !my_car.car_right && lane != 2 ){
      // if there is no car right and there is a right lane.
      cout << "| Change lane to the RIGHT" << endl;
      lane++; // Change lane right.
    } else {
      cout << "| Keep CURRENT lane. Decrease SPEED" << endl;
      speed_diff -= MAX_ACC;
    }
  } else {
    if ( lane != 1 ) { // if we are not on the center lane.
      if ( ( lane == 0 && !my_car.car_right ) || ( lane == 2 && !my_car.car_left ) ) {
        cout << "| Go back to CENTER lane" << endl;
        lane = 1; // Back to center.
      }
    }
    if ( ref_vel < MAX_SPEED ) {
      cout << "| Keep CURRENT lane. Increase SPEED" << endl;
      speed_diff += MAX_ACC;
    }
  }
}

int main() {
  uWS::Hub h;
  Path my_path;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // Car's lane. Stating at middle lane.
  int lane = 1;

  // Reference velocity.
  double ref_vel = 0.0; // mph

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
    my_path.update(x, y, s, d_x, d_y);
  }

  h.onMessage([&ref_vel, &lane, &my_path]
                  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          Vehicle my_car(j[1]);

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          // Provided previous path point size.
          auto prev_size = (int)previous_path_x.size();

          // Preventing collisions.
          if (prev_size > 0) {
            my_car.car_s = end_path_s;
          }

          calculate_prediction(sensor_fusion, my_car, lane, prev_size);

          double speed_diff = 0;
          plan_behaviour(my_car, lane, speed_diff, ref_vel);

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          calculate_trajectory(my_car, prev_size, previous_path_x, previous_path_y,
                               lane, ref_vel, speed_diff, next_x_vals, next_y_vals, my_path);

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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