
#include <MQTTClient.h>
#include "motorDriver.h"
#include "sensorDriver.h"

class logic {
  public:
    logic();
    void reachTarget();                                           /* Reach the specified target */
    void setMotor(mclass motor);
    void setSensor(sclass sensor);
    void updateTarget(String &payload);            /* Update target coordinates */
    void updateRover(String &payload);             /* Update rover coordinates */
  protected:
    void roverReadyToMove();
    void alignCoordinates();
    void rotate(int angle);
    void moveTwoMotors(Direction direction, int time);
    void moveTwoMotors(Direction direction);
    void stopMotors();
    void alignXaxis();
    void alignYaxis();
    void avoidObstacleOnXaxis();
    void avoidObstacleOnYaxis();
    bool checkObstacle();
    int getObstacleDistance();
    int obstacleDirection();
    bool targetReached();
    bool targetXReached();
    bool targetYReached();
    void finalDance();
  private:
    mclass motor;
    sclass sensor;
    int targetX;
    int targetY;
    int roverX;
    int roverY;
    int roverAngle;
    const int BORDER_SIZE = 480;
    const int DEFAULT_AVOID_RUN_DURATION = 8000;
    const int DEFAULT_MOTOR_SPEED = 100;


};

extern logic logicobject;