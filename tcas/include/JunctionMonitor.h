#ifndef JUNCTIONMONITOR_H
#define JUNCTIONMONITOR_H

#include <bits/stdc++.h>
#include <pthread.h>

#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <thread>
#include <vector>
#define MAX 99999

#include "CustomTypes.h"

namespace TCAS {

class JunctionMonitor {
 public:
  std::vector<Eigen::VectorXf> junctions_;
  std::vector<JunctionData> junction_data;
  std::vector<int> robot_queue;
  std::vector<std::string> nav_srv_;
  std::vector<std::string> goal_topics_;

  int no_of_robots;
  double robot_radius;

  void init();
  int junctionMonitor(Robot& robot, int j);
  bool isInsideJunctionRegion(Robot& robot, Eigen::VectorXf region);
  bool checkInside(Point polygon[], int n, Point p);
  bool onLine(Line l1, Point p);
  bool isIntersect(Line l1, Line l2);
  float direction(Point a, Point b, Point c);
};

}  // namespace TCAS
#endif  // JUNCTIONMONITOR_H
