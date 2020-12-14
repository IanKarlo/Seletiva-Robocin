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


class Robot {

  private:
    double x;
    double y;
    double vx;
    double vy;
    bool updated;
    long long id;

  public:
    Robot(SSL_DetectionRobot &robot);
    long long getID();
    bool getUpdated();
    void setUpdated(bool value);
    void updateByValues(SSL_DetectionRobot &robot);
    void updateByVelocity();
    double getX();
    double getY();
};

class Ball {
  private:
    double x;
    double y;
    double vx;
    double vy;
  
  public:
    Ball(SSL_DetectionBall ball);
    void update(SSL_DetectionFrame detection);
    void updateByVelocity();
    double getX();
    double getY();
};