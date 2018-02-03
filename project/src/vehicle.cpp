//
// Created by Olga Oleksyuk on 1/31/18.
//
#include "vehicle.h"
#include "json.hpp"
using json = nlohmann::json;

Vehicle::Vehicle(json::basic_json car_data){
  car_x = car_data["x"];
  car_y = car_data["y"];
  car_s = car_data["s"];
  car_d = car_data["d"];
  car_yaw = car_data["yaw"];
  car_speed = car_data["speed"];
  in_lane = false;
  car_left = false;
  car_right = false;
}