
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
    void rotate(int angle);
    void moveTwoMotors(Direction direction, int time);
    void moveTwoMotors(Direction direction);
    void stopMotors();
    bool checkObstacle();
    int getObstacleDistance();
    void finalDance();
    int angleToTarget();
    void avoidObstacle();
    void obstacleAvoidance(int firstAngleOption, int secondAngleOption);
    int avoidanceMoveTime(int angle);
  private:
    mclass motor;
    sclass sensor;
    int targetX;
    int targetY;
    int roverX;
    int roverY;
    int roverAngle;
    const int BORDER_SIZE = 480;
    const int DEFAULT_AVOID_RUN_DURATION = 4000;
    const int DEFAULT_MOTOR_SPEED = 250;
    const int DEFAULT_ROTATION_SPEED = 70;


};

extern logic logicobject;