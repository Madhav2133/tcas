#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <eigen3/Eigen/Dense>

namespace TCAS {

struct Point {
  float x, y;
};

struct Line {
  Point p1, p2;
};

struct JunctionData {
  std::string junction_name;
  int robot_id = -1;
};

struct Robot {
  unsigned int id = 0;
  Eigen::Vector3f position;
};

// struct Junctions {
//   Eigen::VectorXf junction_coordinates;
// };

}  // namespace TCAS
#endif  // CUSTOMTYPES_H
