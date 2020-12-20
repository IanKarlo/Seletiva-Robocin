#include "Basics.h"
#define DT (1.0/60.0)

/*********************************
    * ---- Robot Methods ---- *
 *********************************/

/*
  Instancia o robô de acordo com os dados do frame e inicializa os filtros de kalman
  considerando que em um os dados recebidos são mais confiáveis que o modelo e no outro
  que os dados do modelo são mais confiáveis que os recebidos. O conceito dos filtros é
  que enquanto você recebe dados do frame, é possivel estimar a posição dos robôs com uma
  certa confiança a partir dele, mas quando não recebe os dados, temos que dar mais confiança
  ao modelo.
*/
Robot::Robot(SSL_DetectionRobot &robot) {
  id = robot.robot_id();

  x = robot.x();
  y = robot.y();

  vx = 0;
  vy = 0;

  int n = 4, m = 2;

  Eigen::MatrixXd A(n, n); // Matriz representativa do sistema.
  Eigen::MatrixXd C(n, n); // Matriz de saídas.
  Eigen::MatrixXd Q(n, n); // Matriz de ruído do modelo.
  Eigen::MatrixXd R(n, n); // Matriz de ruído da medição.
  Eigen::MatrixXd P(n, n); // Matriz de covariância estimada no processo.

  A << 1, 0, DT, 0, 0, 1, 0, DT, 0, 0, 1, 0, 0, 0, 0, 1;
  Q << .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05;
  R << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  C << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  P << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10;

  filter = new KalmanFilter(DT, A, C, Q, R, P); //Filtro considerando que a medição é mais confiável que o modelo.

  Eigen::VectorXd x0(n);

  x0 << x, y, 0, 0;

  filter->init(0, x0); //Inicia o filtro com os valores iniciais.

  Eigen::MatrixXd Q2(n, n); // Matriz de ruído do modelo.
  Eigen::MatrixXd R2(n, n); // Matriz de ruído da medição.
  Eigen::MatrixXd P2(n, n); // Matriz de covariância estimada no processo.

  Q2 << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10;
  R2 << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  P2 << .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05;

  filter2 = new KalmanFilter(DT, A, C, Q2, R2, P2); //Filtro considerando que o modelo é mais confiável que a medição.

  Eigen::VectorXd x02(n);

  x0 << x, y, 0, 0;

  filter2->init(0, x0); //Inicia o filtro com os valores iniciais.

  updated = true;

}

double Robot::getX() {
  return x;
}

double Robot::getVx() {
  return vx;
}

double Robot::getY() {
  return y;
}

double Robot::getVy() {
  return vy;
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

/*
  Atualiza ambos os filtros com os valores obtidos no frame, mas salva como valores de posição
  do robo os valores gerados pelo filtro que considera a medição mais confiável que o modelo.
*/

void Robot::updateByValues(SSL_DetectionRobot &robot) {

    double newX = robot.x();
    double newY = robot.y();

    Eigen::VectorXd y1(4);
    Eigen::VectorXd y2(4);

    y1 << newX, newY, vx, vy;
    y2 << newX, newY, vx, vy;

    filter->update(y1);
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

/*
  Atualiza ambos os filtros com os valores obtidos no frame, mas salva como valores de posição
  do robo os valores gerados pelo filtro que considera o modelo mais confiável que a medição.
*/

void Robot::updateByVelocity() {

  Eigen::VectorXd y1(4);
  Eigen::VectorXd y2(4);

  double predX = x + vx*DT;
  double predY = y + vy*DT;

  y1 << predX, predY, vx, vy;
  y2 << predX, predY, vx, vy;

  filter->update(y1);
  filter2->update(y2);

  Eigen::VectorXd state = filter2->state();

  double newX = state[0];
  double newY = state[1];

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

  Eigen::MatrixXd A(n, n); // Matriz representativa do sistema.
  Eigen::MatrixXd C(n, n); // Matriz de saídas.
  Eigen::MatrixXd Q(n, n); // Matriz de ruído do modelo.
  Eigen::MatrixXd R(n, n); // Matriz de ruído da medição.
  Eigen::MatrixXd P(n, n); // Matriz de covariância estimada no processo.

  A << 1, 0, DT, 0, 0, 1, 0, DT, 0, 0, 1, 0, 0, 0, 0, 1;
  Q << .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05;
  R << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  C << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  P << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10;

  filter = new KalmanFilter(DT, A, C, Q, R, P); //Filtro considerando que a medição é mais confiável que o modelo.

  Eigen::VectorXd x0(n);

  x0 << x, y, 0, 0;

  filter->init(0, x0); //Inicia o filtro com os valores iniciais.

  Eigen::MatrixXd Q2(n, n); // Matriz de ruído do modelo.
  Eigen::MatrixXd R2(n, n); // Matriz de ruído da medição.
  Eigen::MatrixXd P2(n, n); // Matriz de covariância estimada no processo.

  Q2 << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10;
  R2 << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  P2 << .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05, 0, 0, 0, 0, .05;

  filter2 = new KalmanFilter(DT, A, C, Q2, R2, P2); //Filtro considerando que o modelo é mais confiável que a medição.

  Eigen::VectorXd x02(n);

  x0 << x, y, 0, 0;

  filter2->init(0, x0); //Inicia o filtro com os valores iniciais.

}

/*
  Atualiza ambos os filtros com os valores obtidos no frame, mas salva como valores de posição
  da bola os valores gerados pelo filtro que considera a medição mais confiável que o modelo.
*/

void Ball::update(SSL_DetectionFrame &detection) {

  int cntBalls = detection.balls_size();

  double newX = 0, newY = 0;

  for(int i = 0; i < cntBalls; i++) {
    newX += detection.balls(i).pixel_x();
    newY += detection.balls(i).pixel_y();
  }
  newX /= cntBalls;
  newY /= cntBalls;

  Eigen::VectorXd y1(4);
  Eigen::VectorXd y2(4);

  y1 << newX, newY, vx, vy;
  y2 << newX, newY, vx, vy;

  filter->update(y1);
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

/*
  Atualiza ambos os filtros com os valores obtidos no frame, mas salva como valores de posição
  da bola os valores gerados pelo filtro que considera o modelo mais confiável que a medição.
*/

void Ball::updateByVelocity() {

  Eigen::VectorXd y1(4);
  Eigen::VectorXd y2(4);

  double predX = x + vx*DT;
  double predY = y + vy*DT;

  y1 << predX, predY, vx, vy;
  y2 << predX, predY, vx, vy;

  filter->update(y1);
  filter2->update(y2);

  Eigen::VectorXd state = filter2->state();

  double newX = state[0];
  double newY = state[1];

  vx = newX - x;
  vy = newY - y;

  filter->updateVelocity(vx, vy);

  x = newX;
  y = newY;

}

double Ball::getX() {
  return x;
}

double Ball::getVx() {
  return vx;
}

double Ball::getY() {
  return y;
}

double Ball::getVy() {
  return vy;
}