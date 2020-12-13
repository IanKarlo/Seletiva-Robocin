#include <stdio.h>
#include <fstream>
#include <bits/stdc++.h>
#include "pb/messages_robocup_ssl_detection.pb.h"
#include "pb/messages_robocup_ssl_geometry.pb.h"
#include "pb/messages_robocup_ssl_wrapper.pb.h"
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