#include <Eigen/Dense>

#pragma once

class KalmanFilter {

public:
  KalmanFilter(
      double dt,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  ); //Construtor

  void init(); //Inicializador do filtro quando não tem valores iniciais.

  void init(double t0, const Eigen::VectorXd& x0); //Inicializador do filtro quando tem valores iniciais.

  void update(const Eigen::VectorXd& y); //Atualizar o filtro apenas com um novo vetor de leitura.

  void updateVelocity(double vx, double vy); //Atualiza os valores de velocidade no vetor de estados do filtro.

  Eigen::VectorXd state() { return x_hat; }; //Retorna o vetor de estados do filtro.
  
  double time() { return t; }; //Retorna o momento atual do filtro.

private:

  // Matrizes para calculo do filtro.
  Eigen::MatrixXd A, C, Q, R, P, K, P0;

  // Dimensões do sistema.
  int m, n; //

  // Tempo inicial e tempo atual do filtro.
  double t0, t;

  // Tempo de discretização do filtro.
  double dt;

  // Variável para verificação de inicialização do filtro.
  bool initialized;

  // Matriz identidade de tamanho n².
  Eigen::MatrixXd I;

  // Vetores de estados do filtro.
  Eigen::VectorXd x_hat, x_hat_new;
};