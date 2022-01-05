
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
  private:
    mclass motor;
    sclass sensor;
    int targetX;
    int targetY;
    int roverX;
    int roverY;
    int roverAngle;


};

extern logic logicobject;