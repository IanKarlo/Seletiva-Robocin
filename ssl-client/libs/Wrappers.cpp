#include "Wrappers.h"

/*********************************
 * ---- RobotVector Methods ---- *
 *********************************/
/*
  Salva qual é a cor dos robos a se salvar nesse wrapper.
*/
RobotVector::RobotVector(bool color) {
  this->color = color;
}

/*
  Recebe o pacote de detecção e verifica para cada robô, a cor é selecionada pelo atributo setado no construtor,
  se ele já existe no vetor, caso ja exista atualiza com os valores recebidos, caso não exista ele cria, até que
  o vetor tenha no máximo 6 robôs. Por fim ele verifica na lista de robôs se algum não foi atualizado com dados do
  frame, caso exista algum, ele o atualiza com o método para atualizar sem dados.
*/

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
        } else if(ids.size() < 6) {
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
        } else if(ids.size() < 6){
          ids.insert(id);
          Robot newRobot = Robot(robot);
          robots.push_back(newRobot);
        }
      }
    }
  }
  for (int i = 0; i < robots.size(); i++){
    if(robots[i].getUpdated() == false) {
      robots[i].updateByVelocity();
    } else {
      robots[i].setUpdated(false);
    }
  }
}

/*
  Para cada robô, printa no arquivo os valores de posição e velocidade.
*/

void RobotVector::printAll(ofstream &OutFile) {
  for(int i = 0; i < robots.size(); i++) {
    OutFile << std::to_string(robots[i].getID()) + ", " + std::to_string(robots[i].getX()) + ", " + std::to_string(robots[i].getY()) + ", " + std::to_string(robots[i].getVx()) + ", " + std::to_string(robots[i].getVy()) << '\n';
  }
}

/*********************************
 * ---- BallWrapper Methods ---- *
 *********************************/

/*
  Constrói a bola e indica que ainda não foi setado algum valor nela.
*/

BallWrapper::BallWrapper() {
  setted = false;
}

/*
  Caso a bola ainda não tenha recebido algum valor, constroi a mesma, caso já tenha, verifica se existem bolas no frame,
  se existirem bolas, calcula a sua posição com o método que recebe valores, caso não exista, calcula com o método que não
  recebe valores.
*/

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

/*
  Printa no arquivo a posição estimada da bola.
*/

void BallWrapper::printAll(ofstream &OutFile) {
  OutFile << "Ball, " + std::to_string(ball->getX()) + ", " + std::to_string(ball->getY()) + ", " + std::to_string(ball->getVx()) + ", " + std::to_string(ball->getVy()) << '\n';
}
