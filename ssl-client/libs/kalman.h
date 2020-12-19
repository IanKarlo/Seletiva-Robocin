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
  );

  void init();

  void init(double t0, const Eigen::VectorXd& x0);

  void update(const Eigen::VectorXd& y);

  void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

  void updateVelocity(double vx, double vy);

  Eigen::VectorXd state() { return x_hat; };
  
  double time() { return t; };

private:

  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};