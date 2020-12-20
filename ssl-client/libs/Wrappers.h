#include "Basics.h"

class RobotVector {
  private:
    std::vector<Robot> robots; //Vetor com os robôs.
    std::unordered_set<int> ids; //Set com os ids dos robôs para garantir a unicidade de dados e evitar aceitar leituras erradas de outros robos.
    bool color; //0 -> Blue | 1 -> Yellow
  public:
    RobotVector(bool color = 0); //Construtor
    void updateAll(SSL_DetectionFrame &detection); //Atualiza todos os robôs.
    void printAll(ofstream &OutFile); //Salva no arquivo as posições dos robôs.
};

class BallWrapper {
  private:
    Ball* ball; //Bola.
    bool setted; //indicador se a bola está salva.
  public:
    BallWrapper();
    void updateBall(SSL_DetectionFrame &detection);
    void printAll(ofstream &OutFile);
};