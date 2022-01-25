#include "logic.h"
#include <ArduinoJson.h>

// constructor we set the values to be negative just to be able to 
// verify afterwards if they were set from AWS values or not
logic::logic() {
    this->targetX = -1;
    this-> targetY = -1;
    this-> roverX = -1;
    this->roverY = -1;
    this->roverAngle = -1;
}

// to check if a value is inside a range of values 
bool outOfRange(int* validRange, int size, int value) {
    bool outOfRange = true; 
    // Serial.println("outOfRange value:");
    // Serial.println(value);
    for (size_t i = 0; i < size; i++) {
        // Serial.printf("%d,", validRange[i]);
        if (validRange[i] == value)
            outOfRange = false;
    }
    // Serial.println("outOfRange result");
    // Serial.println(outOfRange ? "true" : "false");
    return outOfRange;
}


// checking if coordinates are set ==> rover ready to move
void logic::roverReadyToMove() {
    while (this->targetX == -1 || this->targetY == -1 ||
        this->roverX == -1 || this->roverY == -1 || this->roverAngle == -1 ||
        &this->motor == nullptr || &this->sensor == nullptr) {
        Serial.println("Rover not ready to move");
        // pause for a second
        vTaskDelay(1000);
    }
    
}

// return true if rover X coordinates are equal to target
bool logic::targetXReached() {
    return (targetX - 5 <= roverX && roverX <= targetX + 5) || targetX == -1;
}

// return true if rover Y coordinates are equal to target
bool logic::targetYReached() {
    return (targetY - 5 <= roverY && roverY <= targetY + 5) || targetY == -1;
}

// returns true if rover reached target ==> coordinates are the same with a range of +-5
bool logic::targetReached() {
    return targetYReached() && targetXReached();
}

// where all the logic is (aligning rover with X and Y coordinates of target and avoiding obstacles )
void logic::reachTarget() {
    Serial.println("reachTarget");    
    this->roverReadyToMove();
    // while the rover had not yet reached the target, keep aligning X,Y coordinates with target
    while (!targetReached()) {
        if (!targetXReached())
            this->alignXaxis();
        if (!targetYReached())
            this->alignYaxis();
    }
    finalDance();
}

void logic::finalDance() {
    Serial.println("finalDance");    
}

// deserialize rover data coordinates coming from AWS
void logic::updateRover(String &payload) {
    // {"21": [[126, 43], 276]}
    // Serial.println("updateRover");
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    this->roverX = doc["21"][0][0];
    this->roverY = doc["21"][0][1];
    this->roverAngle = doc["21"][1];
}

// deserialize target data coordinates coming from AWS
void logic::updateTarget(String &payload) {
    // [405, 293]
    // Serial.println("updateTarget");
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    this->targetX = doc[0];
    this->targetY = doc[1];
}

void logic::setMotor(mclass motor) {
    Serial.println("setMotor");
    this->motor = motor;
    this->moveTwoMotors(Forward, 100);
}

// move motor based on direction given then wait for some time and stop motors 
void logic::moveTwoMotors(Direction direction, int time) {
    Serial.printf("moveTwoMotors with direction and time (%d)\n", time);
    if (direction == Forward) {
        this->motor.set_speed(MotorA, Forward, DEFAULT_MOTOR_SPEED);
        this->motor.set_speed(MotorB, Backward, DEFAULT_MOTOR_SPEED);
    } else {
        this->motor.set_speed(MotorA, Backward, DEFAULT_MOTOR_SPEED);
        this->motor.set_speed(MotorB, Forward, DEFAULT_MOTOR_SPEED);
    }
    vTaskDelay(time);
    this->stopMotors();
}

// move motor based on direction given without stoping motors 
void logic::moveTwoMotors(Direction direction) {
    Serial.println("moveTwoMotors with direction");
    if (direction == Forward) {
        this->motor.set_speed(MotorA, Forward, DEFAULT_MOTOR_SPEED);
        this->motor.set_speed(MotorB, Backward, DEFAULT_MOTOR_SPEED);
    } else {
        this->motor.set_speed(MotorA, Backward, DEFAULT_MOTOR_SPEED);
        this->motor.set_speed(MotorB, Forward, DEFAULT_MOTOR_SPEED);
    }
}

// setting speed of motors to 0
void logic::stopMotors() {
    Serial.println("stopMotors");
    this->motor.set_speed(MotorA, Forward, 0);
    this->motor.set_speed(MotorB, Backward, 0);   
}

// aligning rover X coordinate to the target X cordinates
void logic::alignXaxis() {
    Serial.println("alignXaxis");
    // if X coordinate of target is bigger than rover, rotate the rover with 360 angle 
    // the rover will then be parallely aligned with X axis
    if (this->targetX > this->roverX)
        this->rotate(360);
    else
        this->rotate(180);

    // Once the robot rotated , we move it forward to align it with targetX
    this->moveTwoMotors(Forward);
    // as long as the rover isnt yet close to the target
    while (!(this->roverX >= this->targetX-30 && this->roverX <= this->targetX+30)) {
        vTaskDelay(5);
        if (checkObstacle()) {
            this->stopMotors();
            Serial.println("OBSTACLE X !!!");
            //if the obstacle distance is between 0 and 120, we move the rover backward 
            if (this->getObstacleDistance() < 120 && this->getObstacleDistance() > 0)
                this->moveTwoMotors(Backward, DEFAULT_AVOID_RUN_DURATION * ((150.0 - this->getObstacleDistance()) / 150.0));
            // we move the rover away from obstacle on Xaxis
            this->avoidObstacleOnXaxis();
            this->sensor.flush();
            // then we move Motor forward to continue aligning on X axis
            this->moveTwoMotors(Forward);     
        }
    }
    //once rover is close to target, we stop motors
    this->stopMotors();
}

void logic::avoidObstacleOnXaxis() {
    Serial.println("avoidObstacleOnXaxis");
    int distanceToEndBorder = BORDER_SIZE - roverY;
    int distanceToStartBorder = roverY;
    Serial.println(distanceToEndBorder);
    Serial.println(distanceToStartBorder);
    Serial.println(roverAngle);
    // if the rover is looking at the opposite direction of X axis
    if (this->roverAngle >= 110 && this->roverAngle < 250) { // ~180
        if (distanceToEndBorder > distanceToStartBorder) {
            rotate(90);
            moveTwoMotors(Forward, DEFAULT_AVOID_RUN_DURATION);
            rotate(180);   
        } else {
            rotate(270);
            moveTwoMotors(Forward, DEFAULT_AVOID_RUN_DURATION);
            rotate(180);
        }
    } else {
        if (distanceToEndBorder > distanceToStartBorder) {
            rotate(270);
            moveTwoMotors(Forward, DEFAULT_AVOID_RUN_DURATION);
            rotate(360);
        } else {
            rotate(90);
            moveTwoMotors(Forward, DEFAULT_AVOID_RUN_DURATION);
            rotate(360);
        }
    }
    Serial.println("avoidObstacleOnXaxis avoided");
}


    /* 
     this function is used to avoid obstcles on y axis
     if the rover is looking at the obstacle in direction of y axis
     then rotate it away from it, if not then continue to target
    */
void logic::avoidObstacleOnYaxis() {
    Serial.println("avoidObstacleOnYaxis");
    int distanceToEndBorder = BORDER_SIZE - roverX;
    int distanceToStartBorder = roverX;
    Serial.println(distanceToEndBorder);
    Serial.println(distanceToStartBorder);
    Serial.println(roverAngle);
    if (this->roverAngle >= 200 && this->roverAngle <= 340) { // ~270
        if (distanceToEndBorder > distanceToStartBorder) {
            rotate(360);
            moveTwoMotors(Forward, DEFAULT_AVOID_RUN_DURATION);
            rotate(270);   
        } else {
            rotate(180);
            moveTwoMotors(Forward, DEFAULT_AVOID_RUN_DURATION);
            rotate(270);
        }
    } else {
        if (distanceToEndBorder > distanceToStartBorder) {
            rotate(360);
            moveTwoMotors(Forward, DEFAULT_AVOID_RUN_DURATION);
            rotate(90);
        } else {
            rotate(180);
            moveTwoMotors(Forward, DEFAULT_AVOID_RUN_DURATION);
            rotate(90);
        }
    }
    Serial.println("avoidObstacleOnYaxis avoided");
}

    /* 
     this function returns the direction of the obstacle 
     returns 0 for left sensor
     returns 1 for center sensor
     returns 2 for right sensor
     returns 1 for default
    */
int logic::obstacleDirection() {
    Serial.println("obstacleDirection");
    int16_t *arr =  sensorobject.reading(); // left-root-right
    if (arr[0] < 150 && arr[1] > 300 && arr[2] > 300) return 0; // means left
    if (arr[0] < 300 || arr[1] < 150 || arr[2] < 300) return 1; // means center
    if (arr[1] > 300 && arr[1] > 300 && arr[2] < 150) return 2; // means right;
    return 1; // default is center
}

// aligning rover Y coordinate to the target Y cordinates
void logic::alignYaxis() {
   Serial.println("alignYaxis");
    // if Y coordinate of target is bigger than rover, rotate the rover with 270 angle else rotate 90
    // the rover will then be parallely aligned with Y axis
    if (this->targetY > this->roverY)
        this->rotate(270);
    else
        this->rotate(90);

    // Once the robot rotated , we move it forward to align it with targetY
    this->moveTwoMotors(Forward);

    // as long as the rover isnt yet close to the target
    while (!(this->roverY >= this->targetY-30 && this->roverY <= this->targetY+30)) {
        vTaskDelay(5);
        if (checkObstacle()) {
            this->stopMotors();
            Serial.println("OBSTACLE Y !!!");
            //if the obstacle distance is between 0 and 120, we move the rover backward 
            if (this->getObstacleDistance() < 120 && this->getObstacleDistance() > 0)
                this->moveTwoMotors(Backward, DEFAULT_AVOID_RUN_DURATION * ((150.0 - this->getObstacleDistance()) / 150.0));
            // we move the rover away from obstacle on Yaxis
            this->avoidObstacleOnYaxis();\
            this->sensor.flush();
            this->moveTwoMotors(Forward); 
        }
    }

    //once rover is close to target, we stop motors
    this->stopMotors();
}

// read values of sensor and return true if sensor values are less than 120 
bool logic::checkObstacle() {
    int16_t *arr =  sensorobject.reading(); // left-root-right
    // Serial.println("checkObstacle");  
    if (arr[0] < 30 || arr[1] < 30 || arr[2] < 30) return false;
    for(int i = 0; i < 3; i++)
        Serial.printf("%d, ", arr[i]);  
    Serial.println(""); 
    return arr[0] < 120 || arr[1] < 150 || arr[2] < 120;
}

//read sensor values and return the minimum of 3 values
int logic::getObstacleDistance() {
    int16_t *arr =  sensorobject.reading();
    return min(arr[0], min(arr[1], arr[2]));
}

void logic::rotate(int angle) {
    Serial.printf("rotate: %d\n", angle);
    //Move rover back to compensate for rover's movement while rotating
    this->moveTwoMotors(Backward, 500);
    int validRange[11] = {(angle - 5) % 360,(angle - 4) % 360,(angle - 3) % 360, (angle - 2) % 360, (angle - 1) % 360, angle % 360, (angle + 1) % 360, (angle + 2) % 360, (angle + 3) % 360, (angle + 4) % 360, (angle + 5) % 360};
    // if the roverAngle is not out of range
    // meaning that rover's direction mostly parallel to the X axis
    if (!outOfRange(validRange + 1, 9, roverAngle)) return;
    if (((angle - this->roverAngle + 540) % 360) - 180 > 0) { // 360 - 270 = 90 > 0 -> left // 180 - 270 = -90 < 0 -> right // 100 - 200 = -100 -> right
        // rotate left
        // Serial.println("rotate left");
        this->motor.set_speed(MotorB, Backward, DEFAULT_MOTOR_SPEED * 3 / 4);
    } else {
        // Serial.println("rotate right");
        // rotate right
        this->motor.set_speed(MotorA, Forward, DEFAULT_MOTOR_SPEED * 3 / 4);
    }
    // -5 <= roverAngle(266) <= 5
    Serial.println("rotating...");
    while (outOfRange(validRange, 11, this->roverAngle)) {
        // Serial.println("rotating...");
        vTaskDelay(5);
    }
    this->stopMotors();
    Serial.println("rotate end");
}


void logic::setSensor(sclass sensor) {
    Serial.println("setSensor");
    this->sensor = sensor;
}

