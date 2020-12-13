#include "RobotVector.h"

RobotVector::RobotVector(bool color) {
  this->color = color;
}

void RobotVector::updateAll(SSL_DetectionFrame &detection) {
  if(this->color == 0) { // Blue
    int robots_blue_n =  detection.robots_blue_size();
    for (int i = 0; i < robots_blue_n; i++) {
      SSL_DetectionRobot robot = detection.robots_blue(i);
      if(robot.has_robot_id()) {
        int id = robot.robot_id();
        if(ids.find(id) != ids.end()) {
          for(int j = 0; j < robots.size(); j++) {
            if(robots[j].getID() == id) {
              robots[j].updateByValues(robot);
              robots[j].setUpdate(true);
              break;
            }
          }
        } else {
          ids.insert(id);
          robots.push_back(new Robot(robot));
        }
      }
    }   
  } else {
    int robots_yellow_n =  detection.robots_yellow_size();
    for (int i = 0; i < robots_yellow_n; i++) {
      SSL_DetectionRobot robot = detection.robots_yellow(i);
      if(robot.has_robot_id()) {
        int id = robot.robot_id();
        if(ids.find(id) != ids.end()) {
          for(int j = 0; j < robots.size(); j++) {
            if(robots[j].getID() == id) {
              robots[j].updateByValues(robot);
              robots[j].setUpdate(true);
              break;
            }
          }
        } else {
          ids.insert(id);
          robots.push_back(new Robot(robot));
        }
      }
    }
  }
  for (int i = 0; i < robots.size(); i++){
    if(robots[i].getUpdated() == false) {
      robots[i].updateByVelocity();
      robots[i].setUpdate(true);
    }
  }
}

void RobotVector::printAll(ofstream OutFile) {
  for(int i = 0; i < robots.size(); i++) {
    OutFile << std::to_string(robots[i].getID()) + ", " + std::to_string(robots[i].getX()) + ", " + std::to_string(robots[i].getY()) << '\n';
    robots[i].setUpdate(false);
  }
}