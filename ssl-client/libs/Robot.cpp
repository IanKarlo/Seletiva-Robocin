#include "Robot.h"
#define DT (1.0/60.0)

Robot::Robot(SSL_DetectionRobot &robot) {
  this->x = robot.x();
  this->y = robot.y();
  this->vx = 0;
  this->vy = 0;
  this->updated = true;
  this->id = robot.robot_id();
}

double Robot::getX() {
  return this->x;
}

double Robot::getY() {
  return this->y;
}

long long Robot::getID() {
  return this->id;
}

bool Robot::getUpdated() {
  return this->updated;
}

void Robot::setUpdated(bool value) {
  this->updated = value;
}

void Robot::updateByValues(SSL_DetectionRobot &robot) {
  if(robot.confidence() > 0.8) {
    double actualX = robot.x();
    double actualY = robot.y();
    double actualVx = (actualX - this->x)/DT;
    double actualVy = (actualY - this->y)/DT;
    this->x = actualX;
    this->y = actualY;
    this->vx = actualVx;
    this->vy = actualVy;
  } else {
    double firstX = robot.x();
    double secondX = this->x + this->vx*DT;
    double firstY = robot.y();
    double secondY = this->y + this->vx*DT;
    double actualX = (firstX + secondX)/2;
    double actualY = (firstY + secondY)/2;
    double actualVx = (actualX - this->x)/DT;
    double actualVy = (actualY - this->y)/DT;
    this->x = actualX;
    this->y = actualY;
    this->vx = actualVx;
    this->vy = actualVy;
  }
}

void Robot::updateByVelocity() {
  double actualX = this->x + this->vx*DT;
  double actualY = this->y + this->vy*DT;
  double actualVx = (actualX - this->x)/DT;
  double actualVy = (actualY - this->y)/DT;
  this->x = actualX;
  this->y = actualY;
  this->vx = actualVx;
  this->vy = actualVy;
}