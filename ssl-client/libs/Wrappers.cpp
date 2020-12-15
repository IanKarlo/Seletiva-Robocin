#include "Wrappers.h"

/*********************************
 * ---- RobotVector Methods ---- *
 *********************************/

RobotVector::RobotVector(bool color) {
  this->color = color;
}

void RobotVector::updateAll(SSL_DetectionFrame &detection) {
  if(color == 0) { // Blue
    int robots_blue_n =  detection.robots_blue_size();
    for (int i = 0; i < robots_blue_n; i++) {
      SSL_DetectionRobot robot = detection.robots_blue(i);
      if(robot.has_robot_id()) {
        int id = robot.robot_id();
        if(ids.find(id) != ids.end()) {
          for(int j = 0; j < robots.size(); j++) {
            if(robots[j].getID() == id) {
              robots[j].updateByValues(robot);
              robots[j].setUpdated(true);
              break;
            }
          }
        } else {
          ids.insert(id);
          Robot newRobot = Robot(robot);
          robots.push_back(newRobot);
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
              robots[j].setUpdated(true);
              break;
            }
          }
        } else {
          ids.insert(id);
          Robot newRobot = Robot(robot);
          robots.push_back(newRobot);
        }
      }
    }
  }
  for (int i = 0; i < robots.size(); i++){
    if(robots[i].getUpdated() == false) {
      //robots[i].updateByVelocity();
    } else {
      robots[i].setUpdated(false);
    }
  }
}

void RobotVector::printAll(ofstream &OutFile) {
  for(int i = 0; i < robots.size(); i++) {
    OutFile << std::to_string(robots[i].getID()) + ", " + std::to_string(robots[i].getX()) + ", " + std::to_string(robots[i].getY()) << '\n';
  }
}

/*********************************
 * ---- BallWrapper Methods ---- *
 *********************************/

BallWrapper::BallWrapper() {
  setted = false;
}

void BallWrapper::updateBall(SSL_DetectionFrame &detection) {
  if(!setted) {
    int cntBalls = detection.balls_size();
    if(cntBalls <= 0) return;
    SSL_DetectionBall detBall = detection.balls(0);
    ball = new Ball(detBall);
    setted = true;
  } else {
    int cntBalls = detection.balls_size();
    if(cntBalls <= 0) {
      (*ball).updateByVelocity();
    } else {
      (*ball).update(detection);
    }
  }
}

void BallWrapper::printAll(ofstream &OutFile) {
  OutFile << "Ball, " + std::to_string(ball->getX()) + ", " + std::to_string(ball->getY()) << '\n';
}
