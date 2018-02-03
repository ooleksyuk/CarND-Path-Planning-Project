//
// Created by Olga Oleksyuk on 1/31/18.
//

#pragma once

#include "json.hpp"
using json = nlohmann::json;

class Vehicle {
public:
  // Main car's localization Data
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  bool car_ahead;
  bool car_left;
  bool car_right;

  Vehicle(json::basic_json car_data);
};


