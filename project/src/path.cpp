//
// Created by Olga Oleksyuk on 1/31/18.
//

#include "path.h"

Path::Path(){}
void Path::update(double x, double y, double s, double d_x, double d_y){
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  map_waypoints_x.push_back(x);
  map_waypoints_y.push_back(y);
  map_waypoints_s.push_back(s);
  map_waypoints_dx.push_back(d_x);
  map_waypoints_dy.push_back(d_y);
}