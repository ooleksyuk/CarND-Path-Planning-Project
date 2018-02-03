//
// Created by Olga Oleksyuk on 1/31/18.
//
#include "vehicle.h"
#include "json.hpp"
using json = nlohmann::json;

Vehicle::Vehicle(json::basic_json car_data){
  car_x = car_data[1]["x"];
  car_y = car_data[1]["y"];
  car_s = car_data[1]["s"];
  car_d = car_data[1]["d"];
  car_yaw = car_data[1]["yaw"];
  car_speed = car_data[1]["speed"];
  car_ahead = false;
  car_left = false;
  car_right = false;
}