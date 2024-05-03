#include "JunctionMonitor.h"

namespace TCAS {

void JunctionMonitor::init() { std::cout << "Initiating TCAS" << std::endl; }

int JunctionMonitor::junctionMonitor(TCAS::Robot& robot, int j) {
  int no_of_junctions = junctions_.size();
  junction_data.resize(no_of_junctions);

  int robot_id = robot.id;

  // for (size_t j = 0; j < junctions_.size(); j++) {
  // std::cout << "hi- " << j << std::endl;
  if (isInsideJunctionRegion(robot, junctions_[j])) {
    if (junction_data[j].robot_id == -1) {
      junction_data[j].robot_id = robot_id;
      std::cout << "Robot-" << robot_id << " has entered Junction-" << j + 1
                << std::endl;

      return 1;
    }

    else if (junction_data[j].robot_id != -1 &&
             junction_data[j].robot_id != robot_id) {
      int existing_robot_id = junction_data[j].robot_id;
      std::cout << "Already a robot-" << existing_robot_id
                << " is present in Junction-" << j + 1
                << ". Stopping the current robot-" << robot_id << "."
                << std::endl;

      bool already_present = false;
      if (!robot_queue.empty()) {
        for (int id : robot_queue) {
          if (id == robot_id) {
            already_present = true;
          }
        }
      }

      if (!already_present) {
        robot_queue.push_back(robot_id);
      }

      return 2;
    }
  }

  else {
    if (junction_data[j].robot_id == robot_id) {
      // Robot is not in the junction, so we need to empty the array.
      std::cout << "Robot-" << robot_id << " has exited Junction-" << j + 1
                << std::endl;
      if (!robot_queue.empty()) {
        junction_data[j].robot_id = robot_queue.front();
      }

      else {
        junction_data[j].robot_id = -1;
      }

      return 3;
    }

    return 4;
  }
  // }
  return 0;
}

bool JunctionMonitor::isInsideJunctionRegion(Robot& robot,
                                             Eigen::VectorXf region) {
  int n = region.size() / 2;

  float robot_x = robot.position(0);
  float robot_y = robot.position(1);

  Point p = {robot_x, robot_y};

  Point polygon[] = {{region(0), region(1)},
                     {region(2), region(3)},
                     {region(4), region(5)},
                     {region(6), region(7)}};

  if (checkInside(polygon, n, p)) {
    return true;
  }

  return false;
}

bool JunctionMonitor::checkInside(Point polygon[], int n, Point p) {
  if (n < 3) {
    std::cout << "Error, not a polygon!" << std::endl;
    return 0;
  }

  Line exline = {p, {9999, p.y}};
  int count = 0;
  int i = 0;
  do {
    Line side = {polygon[i], polygon[(i + 1) % n]};
    if (isIntersect(side, exline)) {
      if (direction(side.p1, p, side.p2) == 0) {
        return onLine(side, p);
      }
      count++;
    }
    i = (i + 1) % n;
  } while (i != 0);

  return count & 1;
}

bool JunctionMonitor::onLine(Line l1, Point p) {
  if (p.x <= std::max(l1.p1.x, l1.p2.x) && p.x <= std::min(l1.p1.x, l1.p2.x) &&
      (p.y <= std::max(l1.p1.y, l1.p2.y) && p.y <= std::min(l1.p1.y, l1.p2.y)))
    return true;

  return false;
}

bool JunctionMonitor::isIntersect(Line l1, Line l2) {
  float dir1 = direction(l1.p1, l1.p2, l2.p1);
  float dir2 = direction(l1.p1, l1.p2, l2.p2);
  float dir3 = direction(l2.p1, l2.p2, l1.p1);
  float dir4 = direction(l2.p1, l2.p2, l1.p2);

  if (dir1 != dir2 && dir3 != dir4) return true;

  if (dir1 == 0 && onLine(l1, l2.p1)) return true;

  if (dir2 == 0 && onLine(l1, l2.p2)) return true;

  if (dir3 == 0 && onLine(l2, l1.p1)) return true;

  if (dir4 == 0 && onLine(l2, l1.p2)) return true;

  return false;
}

float JunctionMonitor::direction(Point a, Point b, Point c) {
  float val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);

  if (val == 0)
    return 0;

  else if (val < 0)
    return 2;

  return 1;
}
}  // namespace TCAS