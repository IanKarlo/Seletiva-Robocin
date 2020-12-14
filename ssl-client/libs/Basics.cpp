#include "Basics.h"
#define DT (1.0/60.0)

/*********************************
    * ---- Robot Methods ---- *
 *********************************/

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

/*********************************
    * ---- Ball Methods ---- *
 *********************************/

Ball::Ball(SSL_DetectionBall ball) {
  this->x = ball.pixel_x();
  this->y = ball.pixel_y();
  this->vx = 0;
  this->vy = 0;
}

void Ball::update(SSL_DetectionFrame detection) {
  int cntBalls = detection.balls_size();
  double newX = 0, newY = 0;
  for(int i = 0; i < cntBalls; i++) {
    newX += detection.balls(i).pixel_x();
    newY += detection.balls(i).pixel_y();
  }
  newX /= cntBalls;
  newY /= cntBalls;

  double predX = this->x + this->vx*DT;
  double predY = this->y + this->vy*DT;

  newX += predX;
  newX /= 2;

  newY += predY;
  newY /= 2;

  double newVX = (newX - this->x)/DT;
  double newVY = (newY - this->y)/DT;
  
  this->x = newX;
  this->y = newY;
  this->vx = newVX;
  this->vy = newVY;
}

void Ball::updateByVelocity() {

  double newX = this->x + this->vx*DT;
  double newY = this->y + this->vy*DT;
  double newVX = (newX - this->x)/DT;
  double newVY = (newY - this->y)/DT;
  
  this->x = newX;
  this->y = newY;
  this->vx = newVX;
  this->vy = newVY;
}

double Ball::getX() {
  return this->x;
}

double Ball::getY() {
  return this->y;
}