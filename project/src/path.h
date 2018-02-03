//
// Created by Olga Oleksyuk on 1/31/18.
//

#pragma once
#include <vector>
using namespace std;

class Path {
public:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  Path();
  void update(double x, double y, double s, double d_x, double d_y);
};


