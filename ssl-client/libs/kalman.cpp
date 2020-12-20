#include <iostream>
#include <stdexcept>

#include "kalman.h"
/*
  Inicializa todas as matrizes e variaveis do filtro.
*/
KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

/*
  Inicializa o vetor de estados, as variáveis de tempo e a matriz de covariância do filtro.
*/

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

/*
  Atualiza o filtro com base no vetor de leitura que é passado para o filtro, aplicando as equações propostas pelo filtro de Kalman.
*/

void KalmanFilter::update(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  t += dt;
}

/*
  Atualiza os valores de velocidade do vetor de estados do filtro.
*/

void KalmanFilter::updateVelocity(double vx, double vy) {
  x_hat[2] = vx;
  x_hat[3] = vy;
}


/*
  * A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1; -> Matriz do sistema
  * Q << .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05;
  * R << 10, 0, 0, 10;
  * C|H| << 1, 0, 0, 1, 0, 0, 0, 0;
  * P << 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10;
*/