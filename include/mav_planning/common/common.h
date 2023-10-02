#ifndef MAV_PLANNING_COMMON_COMMON
#define MAV_PLANNING_COMMON_COMMON

#include <iostream>
#include <stdarg.h>
#include <memory>
#include <vector>
#include <fstream>
#include <math.h>
#include <Eigen/Eigen>
#include <utility>
#include <string>
#include <random>
#include <map>
#include <chrono>
#include <mutex> 

typedef Eigen::Vector3d Point;
typedef Eigen::Vector4d FlatMAVState;
typedef Eigen::Quaterniond Quaternion;
typedef std::vector<Point> PointList;
typedef std::vector<FlatMAVState> FlatMAVStateList;

struct SE3State
{
    Point position;
    Quaternion rotation;
};

typedef std::vector<SE3State> SE3StateList;


#endif /* MAV_PLANNING_COMMON_COMMON */
