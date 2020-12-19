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

  int n = 4, m = 2;

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(n, m); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  A << 1, 0, DT, 0, 0, 1, 0, DT, 0, 0, 1, 0, 0, 0, 0, 1;
  Q << .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05;
  R << 10, 0, 0, 10;
  C << 1, 0, 0, 1, 0, 0, 0, 0;
  P << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10;

  filter = new KalmanFilter(DT, A, C, Q, R, P);

  Eigen::VectorXd x0(n);

  x0 << x, y, 0, 0;

  filter->init(0, x0);

  Eigen::MatrixXd Q2(n, n); // Process noise covariance
  Eigen::MatrixXd R2(m, m); // Measurement noise covariance
  Eigen::MatrixXd P2(n, n); // Estimate error covariance

  Q2 << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10;
  R2 << 10, 0, 0, 10;
  P2 << .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05;

  filter2 = new KalmanFilter(DT, A, C, Q2, R2, P2);

  Eigen::VectorXd x0(n);

  x0 << x, y, 0, 0;

  filter2->init(0, x0);

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

    double newX = robot.x();
    double newY = robot.y();

    Eigen::VectorXd y(2);
    Eigen::VectorXd y2(2);

    y << newX, newY;
    y2 << newX, newY;

    filter->update(y);
    filter2->update(y2);

    Eigen::VectorXd state = filter->state();

    newX = state[0];
    newY = state[1];

    vx = newX - x;
    vy = newY - y;

    filter->updateVelocity(vx, vy);

    x = newX;
    y = newY;

}

void Robot::updateByVelocity() {

  Eigen::VectorXd y(2);
  Eigen::VectorXd y2(2);

  int predX = x + vx*DT;
  int predY = y + vy*DT;

  y << predX, predY;
  y2 << predX, predY;

  filter->update(y);
  filter2->update(y2);

  Eigen::VectorXd state = filter2->state();

  newX = state[0];
  newY = state[1];

  vx = newX - x;
  vy = newY - y;

  filter->updateVelocity(vx, vy);

  x = newX;
  y = newY;


}

/*********************************
    * ---- Ball Methods ---- *
 *********************************/

Ball::Ball(SSL_DetectionBall &ball) {

  x = ball.pixel_x();
  y = ball.pixel_y();

  vx = 0;
  vy = 0;

  int n = 4, m = 2;

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(n, m); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  A << 1, 0, DT, 0, 0, 1, 0, DT, 0, 0, 1, 0, 0, 0, 0, 1;
  Q << .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05;
  R << 10, 0, 0, 10;
  C << 1, 0, 0, 1, 0, 0, 0, 0;
  P << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10;

  filter = new KalmanFilter(DT, A, C, Q, R, P);

  Eigen::VectorXd x0(n);

  x0 << x, y, 0, 0;

  filter->init(0, x0);

  Eigen::MatrixXd Q2(n, n); // Process noise covariance
  Eigen::MatrixXd R2(m, m); // Measurement noise covariance
  Eigen::MatrixXd P2(n, n); // Estimate error covariance

  Q2 << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10;
  R2 << 10, 0, 0, 10;
  P2 << .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05;

  filter2 = new KalmanFilter(DT, A, C, Q2, R2, P2);

  Eigen::VectorXd x0(n);

  x0 << x, y, 0, 0;

  filter2->init(0, x0);

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

  Eigen::VectorXd y(2);
  Eigen::VectorXd y2(2);

  y << newX, newY;
  y2 << newX, newY;

  filter->update(y);
  filter2->update(y2);

  Eigen::VectorXd state = filter->state();

  newX = state[0];
  newY = state[1];

  vx = newX - x;
  vy = newY - y;

  filter->updateVelocity(vx, vy);

  x = newX;
  y = newY;

}

void Ball::updateByVelocity() {

  Eigen::VectorXd y(2);
  Eigen::VectorXd y2(2);

  int predX = x + vx*DT;
  int predY = y + vy*DT;

  y << predX, predY;
  y2 << predX, predY;

  filter->update(y);
  filter2->update(y2);

  Eigen::VectorXd state = filter2->state();

  newX = state[0];
  newY = state[1];

  vx = newX - x;
  vy = newY - y;

  filter->updateVelocity(vx, vy);

  x = newX;
  y = newY;

}

double Ball::getX() {
  return x;
}

double Ball::getY() {
  return y;
}