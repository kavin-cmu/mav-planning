#ifndef INCLUDE_MAV_PLANNING_COMMON
#define INCLUDE_MAV_PLANNING_COMMON

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <math.h>
#include <Eigen/Eigen>
#include <utility>
#include <string>
#include <random>
#include <map>
#include <chrono>

typedef Eigen::Vector3d Point;
typedef Eigen::Vector4d FlatMAVState;
typedef Eigen::Quaterniond Quaternion;
typedef std::vector<Point> PointList;
typedef std::vector<FlatMAVState> FlatMAVStateList;

#endif /* INCLUDE_MAV_PLANNING_COMMON */
