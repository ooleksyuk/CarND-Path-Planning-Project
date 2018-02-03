/*
 * =====================================================================================
 *
 *       Filename:  cost.h
 *
 *    Description:  Olga Oleksyuk
 *
 *        Version:  1.0
 *        Created:  01/23/2018 14:35:56
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  OLGA OLEKSYUK (olgawow), mail@olga-v.com
 *   Organization:  
 *
 * =====================================================================================
 */
#ifndef COST_H
#define COST_H
#include <vector>

float goal_distance_cost(int goal_lane, int intended_lane, int final_lane, float distance_to_goal);
float inefficiency_cost(int target_speed, int intended_lane, int final_lane, std::vector<int> lane_speeds);

#endif
