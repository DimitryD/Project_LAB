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
        // pause for a 2 seconds
        vTaskDelay(2000);
    }
    
}

// where all the logic is (aligning rover with X and Y coordinates of target and avoiding obstacles )
void logic::reachTarget() {
    Serial.println("reachTarget");    
    this->roverReadyToMove();
    // while the rover had not yet reached the target, keep aligning X,Y coordinates with target
    while (targetX != -1 && targetY != -1) {
        rotate(angleToTarget());
        moveTwoMotors(Forward);
        // 40 basec on delay
        while (abs(targetY - roverY) >= 40 || abs(targetX - roverX) >= 40) {
            if (checkObstacle()) {
                Serial.println("Obstacle detected");   
                stopMotors();
                vTaskDelay(1500); // wait for actual rover coordinates
                avoidObstacle();
                rotate(angleToTarget());
            }
            if (abs(angleToTarget() - roverAngle) > 15) {
                Serial.println("Adjust angle");
                stopMotors();
                rotate(angleToTarget());
                moveTwoMotors(Forward); 
            }
            vTaskDelay(100);
            // int newAngle = (roverAngle + angleToTarget());
            // Serial.println("New angle: ");
            // Serial.println(roverAngle);
            // Serial.println(newAngle);
            // Serial.println(currAngle);
            // if (roverAngle - newAngle > 0)
            // {
            //     this->motor.set_speed(MotorA, Forward, DEFAULT_MOTOR_SPEED - 200);
            //     this->motor.set_speed(MotorB, Backward, DEFAULT_MOTOR_SPEED - 100);
            // }
            // else if (roverAngle - newAngle < 0)
            // {
            //     this->motor.set_speed(MotorA, Forward, DEFAULT_MOTOR_SPEED - 100);
            //     this->motor.set_speed(MotorB, Backward, DEFAULT_MOTOR_SPEED - 200);
            // }
            // else
            // {
            //     this->motor.set_speed(MotorA, Forward, DEFAULT_MOTOR_SPEED);
            //     this->motor.set_speed(MotorB, Backward, DEFAULT_MOTOR_SPEED);
            // }
            // currAngle = newAngle;

        }
        Serial.println("Target reached");
        stopMotors(); // target should be reached
        vTaskDelay(1500); // wait for a new target
    }
    finalDance();
}

// calculate angle from rover to target
int logic::angleToTarget() {
    int deltaY = targetY - roverY;
    int deltaX = targetX - roverX;
    // Serial.println("deltaY:");
    // Serial.println(deltaY);  
    // Serial.println("deltaX:"); 
    // Serial.println(deltaX); 
    int angleAlptha = atan2(abs(deltaY), abs(deltaX)) * 180 / PI;
    int angle = 0;
    // Serial.println("angleAlptha:"); 
    // Serial.println(angleAlptha); 
    if (deltaX > 0 && deltaY > 0) angle = 360 - angleAlptha;
    if (deltaX < 0 && deltaY < 0) angle = 180 - angleAlptha;
    if (deltaX > 0 && deltaY < 0) angle = 0 + angleAlptha;
    if (deltaX < 0 && deltaY > 0) angle = 180 + angleAlptha;
    // Serial.println("Degree:"); 
    // Serial.println(angle);
    return angle;
} 

// determine options to avoid obstacle and avoid it
void logic::avoidObstacle() {
    Serial.println("avoidObstacle");
    int deltaY = targetY - roverY;
    int deltaX = targetX - roverX;
    Serial.println("deltaX:"); 
    Serial.println(deltaX); 
    Serial.println("deltaY:");
    Serial.println(deltaY);  

    // first quadrant
    if (deltaX < 0 && deltaY > 0) {
        obstacleAvoidance(180, 270);
    }
    // second quadrant
    if (deltaX < 0 && deltaY < 0) {
        obstacleAvoidance(90, 180);
    }
    // third quadrant
    if (deltaX > 0 && deltaY < 0) {
        obstacleAvoidance(0, 90);
    }
    // fourth quadrant
    if (deltaX > 0 && deltaY > 0) {
        obstacleAvoidance(270, 0);
    }
}

// avoid obstacle by detecting where to go
void logic::obstacleAvoidance(int firstAngleOption, int secondAngleOption) {
    Serial.println("avoidObstacle");
    rotate(firstAngleOption);
    int firstOptionDistance = getObstacleDistance();
    Serial.println(firstOptionDistance);
    if (firstOptionDistance > 500) 
        moveTwoMotors(Forward, avoidanceMoveTime(firstAngleOption));
    else {
        rotate(secondAngleOption);
        int secondOptionDistance = getObstacleDistance();
        Serial.println(secondOptionDistance);
        if (firstOptionDistance < secondOptionDistance) 
            moveTwoMotors(Forward, avoidanceMoveTime(secondAngleOption));
        else {
            rotate(firstAngleOption);
            moveTwoMotors(Forward, avoidanceMoveTime(firstAngleOption));
        }
    }
    sensor.flush();
}

// determine how much time can we move to not go out of borders
int logic::avoidanceMoveTime(int angle) {
    switch (angle) {
    case 0:
        return DEFAULT_AVOID_RUN_DURATION * ((double)(BORDER_SIZE - roverX) / BORDER_SIZE);
    case 90:
        return DEFAULT_AVOID_RUN_DURATION * ((double)roverY / BORDER_SIZE);
    case 180:
        return DEFAULT_AVOID_RUN_DURATION * ((double)roverX / BORDER_SIZE);
    case 270:
        return DEFAULT_AVOID_RUN_DURATION * ((double)(BORDER_SIZE - roverY) / BORDER_SIZE);
    
    default:
        return 0;
    }
    return 0;
}

// final dance called when all targets reached
void logic::finalDance() {
    Serial.println("finalDance");
    for (int i = 0; i < 10; i++) {
        if (i % 2 == 0) moveTwoMotors(Forward); 
        else moveTwoMotors(Backward); 
        delay(200);
    }
    stopMotors();
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

// motor setup
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

// read values of sensor and return true if sensor values are less than 120 
bool logic::checkObstacle() {
    int16_t *arr =  sensorobject.reading(); // left-root-right
    Serial.println("checkObstacle");  
    if (arr[0] < 30 || arr[1] < 30 || arr[2] < 30) return false;
    for(int i = 0; i < 3; i++)
        Serial.printf("%d, ", arr[i]);  
    Serial.println(""); 
    return arr[0] < 100 || arr[1] < 150 || arr[2] < 100;
}

//read sensor values and return the minimum of 3 values
int logic::getObstacleDistance() {
    sensor.flush();
    int16_t *arr =  sensorobject.reading();
    return min(arr[0], min(arr[1], arr[2]));
}

// rotate to given degree based on camera data
void logic::rotate(int angle) {
    Serial.printf("rotate: %d\n", angle);
    int validRange[11] = {(angle - 5) % 360,(angle - 4) % 360,(angle - 3) % 360, (angle - 2) % 360, (angle - 1) % 360, angle % 360, (angle + 1) % 360, (angle + 2) % 360, (angle + 3) % 360, (angle + 4) % 360, (angle + 5) % 360};
    // if the roverAngle is not out of range
    // meaning that rover's direction mostly parallel to the X axis
    if (!outOfRange(validRange + 1, 11, roverAngle)) return;
    if (((angle - this->roverAngle + 540) % 360) - 180 > 0) { // 360 - 270 = 90 > 0 -> left // 180 - 270 = -90 < 0 -> right // 100 - 200 = -100 -> right
        // rotate left
        // Serial.println("rotate left");
        this->motor.set_speed(MotorB, Backward, DEFAULT_ROTATION_SPEED);
        this->motor.set_speed(MotorA, Backward, DEFAULT_ROTATION_SPEED);
        angle = (360 + angle - 20) % 360; // beacuse of camera delay
    } else {
        // Serial.println("rotate right");
        // rotate right
        this->motor.set_speed(MotorA, Forward, DEFAULT_ROTATION_SPEED);
        this->motor.set_speed(MotorB, Forward, DEFAULT_ROTATION_SPEED);
        angle = (360 + angle + 20) % 360; // beacuse of camera delay
    }
    int validRangeDelayed[11] = {(angle - 5) % 360,(angle - 4) % 360,(angle - 3) % 360, (angle - 2) % 360, (angle - 1) % 360, angle % 360, (angle + 1) % 360, (angle + 2) % 360, (angle + 3) % 360, (angle + 4) % 360, (angle + 5) % 360};
    // -5 <= roverAngle(266) <= 5
    Serial.println("rotating...");
    while (outOfRange(validRangeDelayed, 11, this->roverAngle) && roverAngle != -1) {
        // Serial.println("rotating...");
        vTaskDelay(5);
    }
    this->stopMotors();
    Serial.println("rotate end");
}

// setup sensor
void logic::setSensor(sclass sensor) {
    Serial.println("setSensor");
    this->sensor = sensor;
    sensor.flush();   
}

