#include "Basics.h"
#define DT (1.0/60.0)

/*********************************
    * ---- Robot Methods ---- *
 *********************************/

Robot::Robot(SSL_DetectionRobot &robot) {
  id = robot.robot_id();

  x = robot.x();
  y = robot.y();

  vx = 0;
  vy = 0;

  updated = true;

}

double Robot::getX() {
  return x;
}

double Robot::getY() {
  return y;
}

long long Robot::getID() {
  return id;
}

bool Robot::getUpdated() {
  return updated;
}

void Robot::setUpdated(bool value) {
  updated = value;
}

void Robot::updateByValues(SSL_DetectionRobot &robot) {

    double actualVx = (robot.x() - x)/DT;
    double actualVy = (robot.y() - y)/DT;

    x = robot.x();
    y = robot.y();

    vx = actualVx;
    vy = actualVy;

}

void Robot::updateByVelocity() {

  double actualX = x + vx*DT;
  double actualY = y + vy*DT;

  x = actualX;
  y = actualY;

}

/*********************************
    * ---- Ball Methods ---- *
 *********************************/

Ball::Ball(SSL_DetectionBall &ball) {

  x = ball.pixel_x();
  y = ball.pixel_y();

  vx = 0;
  vy = 0;

}

void Ball::update(SSL_DetectionFrame &detection) {

  int cntBalls = detection.balls_size();

  double newX = 0, newY = 0;

  for(int i = 0; i < cntBalls; i++) {
    newX += detection.balls(i).pixel_x();
    newY += detection.balls(i).pixel_y();
  }
  newX /= cntBalls;
  newY /= cntBalls;

  double predX = x + vx*DT;
  double predY = y + vy*DT;

  newX += predX;
  newX /= 2;

  newY += predY;
  newY /= 2;

  double newVX = (newX - x)/DT;
  double newVY = (newY - y)/DT;
  
  x = newX;
  y = newY;
  vx = newVX;
  vy = newVY;

}

void Ball::updateByVelocity() {

  double newX = x + vx*DT;
  double newY = y + vy*DT;
  
  x = newX;
  y = newY;

}

double Ball::getX() {
  return x;
}

double Ball::getY() {
  return y;
}