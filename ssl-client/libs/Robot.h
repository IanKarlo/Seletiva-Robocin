
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
}