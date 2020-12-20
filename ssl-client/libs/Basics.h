#include <QtNetwork>
#include <stdio.h>
#include <fstream>
#include <bits/stdc++.h>

#include "../net/robocup_ssl_client.h"
#include "../net/grSim_client.h"
#include "../util/timer.h"
#include "../pb/messages_robocup_ssl_detection.pb.h"
#include "../pb/messages_robocup_ssl_geometry.pb.h"
#include "../pb/messages_robocup_ssl_wrapper.pb.h"
#include "../pb/grSim_Packet.pb.h"
#include "../pb/grSim_Commands.pb.h"
#include "../pb/grSim_Replacement.pb.h"
#include "kalman.h"


class Robot {

  private:
    double x; //Posição x do robô.
    double y; //Posição y do robô.
    double vx; //Velocidade do robô no eixo x.
    double vy; //Velocidade do robô no eixo y.
    bool updated; //indicador se o robô já foi atualizado, com o intuito de saber se recebeu algum dado de sua posição no frame.
    long long id;  //ID do robô.
    KalmanFilter* filter; //Filtro de Kalman considerando que o robô recebeu um dado de posição no frame.
    KalmanFilter* filter2; //Filtro de Kalman considerando que o robô não recebeu um dado de posição no frame.

  public:
    Robot(SSL_DetectionRobot &robot); //Construtor.
    long long getID(); //Retorna o ID do robô.
    bool getUpdated(); //Retorna o valor de updated.
    void setUpdated(bool value);  //Seta o valor de updated.
    void updateByValues(SSL_DetectionRobot &robot);  //Atualiza os valores do robô com base nos dados recebidos no frame.
    void updateByVelocity(); //Atualiza os valores do robô com base no modelo criado para o sistema.
    /* -> Métodos de retorno de valores <- */
    double getX();
    double getVx();
    double getY();
    double getVy();
};

class Ball {
  private:
    double x; //Posição x da bola.
    double y; //Posição y da bola.
    double vx; //Velocidade da bola no eixo x.
    double vy; //Velocidade da bola no eixo y.
    KalmanFilter* filter; //Filtro de Kalman considerando que a bola recebeu um dado de posição no frame.
    KalmanFilter* filter2; //Filtro de Kalman considerando que a bola não recebeu um dado de posição no frame.
  
  public:
    Ball(SSL_DetectionBall &ball); //Construtor
    void update(SSL_DetectionFrame &detection); //Atualiza os valores da bola com base nos dados recebidos no frame
    void updateByVelocity(); //Atualiza os valores da bola com base no modelo craido para o sistema
    /* -> Métodos de retorno de valores <- */
    double getX();
    double getVx();
    double getY();
    double getVy();
};