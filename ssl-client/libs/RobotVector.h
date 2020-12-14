#include "Robot.h"

class RobotVector {
  private:
    std::vector<Robot> robots;
    std::unordered_set<long long> ids;
    bool color; //0 -> Blue | 1 -> Yellow
  public:
    RobotVector(bool color = 0);
    void updateAll(SSL_DetectionFrame &detection);
    void printAll();
}