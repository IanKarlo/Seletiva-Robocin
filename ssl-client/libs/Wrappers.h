#include "Basics.h"

class RobotVector {
  private:
    std::vector<Robot> robots;
    std::unordered_set<int> ids;
    bool color; //0 -> Blue | 1 -> Yellow
  public:
    RobotVector(bool color = 0);
    void updateAll(SSL_DetectionFrame &detection);
    void printAll(ofstream &OutFile);
};

class BallWrapper {
  private:
    Ball* ball;
    bool setted;
  public:
    BallWrapper();
    void updateBall(SSL_DetectionFrame &detection);
    void printAll(ofstream &OutFile);
};