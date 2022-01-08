#include "logic.h"
#include <ArduinoJson.h>

logic::logic() {
    this->targetX = -1;
    this-> targetY = -1;
    this-> roverX = -1;
    this->roverY = -1;
    this->roverAngle = -1;
}

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


void logic::roverReadyToMove() {
    while (this->targetX == -1 || this->targetY == -1 ||
        this->roverX == -1 || this->roverY == -1 || this->roverAngle == -1 ||
        &this->motor == nullptr || &this->sensor == nullptr) {
        Serial.println("Rover not ready to move");
        vTaskDelay(1000);
    }
    
}

bool logic::targetXReached() {
    return (targetX - 5 <= roverX && roverX <= targetX + 5) || targetX == -1;
}

bool logic::targetYReached() {
    return (targetY - 5 <= roverY && roverY <= targetY + 5) || targetY == -1;
}

bool logic::targetReached() {
    return targetYReached() && targetXReached();
}

void logic::reachTarget() {
    Serial.println("reachTarget");    
    this->roverReadyToMove();
    while (!targetReached()) {
        if (!targetXReached())
            this->alignXaxis();
        if (!targetYReached())
            this->alignYaxis();
    }
    finalDance();
}

void logic::finalDance() {

}

void logic::updateRover(String &payload) {
    // {"21": [[126, 43], 276]}
    // Serial.println("updateRover");
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    this->roverX = doc["21"][0][0];
    this->roverY = doc["21"][0][1];
    this->roverAngle = doc["21"][1];
}

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

const int DEFAULT_MOTOR_SPEED = 100;

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

void logic::stopMotors() {
    Serial.println("stopMotors");
    this->motor.set_speed(MotorA, Forward, 0);
    this->motor.set_speed(MotorB, Backward, 0);   
}

void logic::alignXaxis() {
    Serial.println("alignXaxis");
    if (this->targetX > this->roverX)
        this->rotate(360);
    else
        this->rotate(180);
    int validRange[11] = {this->targetX-5,this->targetX-4,this->targetX-3,this->targetX-2,this->targetX-1,this->targetX,this->targetX+1,this->targetX+2,this->targetX+3,this->targetX+4,this->targetX+5};
    this->moveTwoMotors(Forward);
    while (!(this->roverX >= validRange[0] && this->roverX <= validRange[10])) {
        vTaskDelay(5);
        if (checkObstacle()) {
            this->stopMotors();
            Serial.println("OBSTACLE X !!!");
            this->avoidObstacleOnXaxis();
        }
    }
    this->stopMotors();
}

const int BORDER_SIZE = 480;
const int DEFAULT_AVOID_RUN_DURATION = 7000;

void logic::avoidObstacleOnXaxis() {
    Serial.println("avoidObstacleOnXaxis");
    int distanceToEndBorder = BORDER_SIZE - targetX;
    int distanceToStartBorder = targetX;
    if (this->roverAngle >= 175 && this->roverAngle < 185) {
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

void logic::avoidObstacleOnYaxis() {
    
}

int logic::obstacleDirection() {
    Serial.println("obstacleDirection");
    int16_t *arr =  sensorobject.reading(); // left-root-right
    if (arr[0] < 150 && arr[1] > 300 && arr[2] > 300) return 0; // means left
    if (arr[0] < 300 || arr[1] < 150 || arr[2] < 300) return 1; // means center
    if (arr[1] > 300 && arr[1] > 300 && arr[2] < 150) return 2; // means right;
    return 1; // default is center
}

void logic::alignYaxis() {
   Serial.println("alignYaxis");
    if (this->targetY > this->roverY)
        this->rotate(270);
    else
        this->rotate(90);
    int validRange[11] = {this->targetY-5,this->targetY-4,this->targetY-3,this->targetY-2,this->targetY-1,this->targetY,this->targetY+1,this->targetY+2,this->targetY+3,this->targetY+4,this->targetY+5};
    this->moveTwoMotors(Forward);
    while (!(this->roverY >= validRange[0] && this->roverY <= validRange[10])) {
        vTaskDelay(5);
        if (checkObstacle()) {
            this->stopMotors();
            Serial.println("OBSTACLE X !!!");
            this->avoidObstacleOnYaxis();\
            checkObstacle();
            vTaskDelay(100);
            checkObstacle();
        }
    }
    this->stopMotors();
}

bool logic::checkObstacle() {
    int16_t *arr =  sensorobject.reading(); // left-root-right
    // Serial.println("checkObstacle");  
    if (arr[0] < 30 || arr[1] < 30 || arr[2] < 30) return false;
    for(int i = 0; i < 3; i++)
        Serial.printf("%d, ", arr[i]);  
    Serial.println(""); 
    return arr[0] < 150 || arr[1] < 150 || arr[2] < 150;
}

void logic::rotate(int angle) {
    Serial.printf("rotate: %d\n", angle);
    this->moveTwoMotors(Backward, 300);
    int validRange[7] = {(angle - 3) % 360, (angle - 2) % 360, (angle - 1) % 360, angle % 360, (angle + 1) % 360, (angle + 2) % 360, (angle + 3) % 360,};
    if (!outOfRange(validRange, 7, roverAngle)) return;
    if (((angle - this->roverAngle + 540) % 360) - 180 > 0) { // 360 - 270 = 90 > 0 -> left // 180 - 270 = -90 < 0 -> right // 100 - 200 = -100 -> right
        // rotate left
        // Serial.println("rotate left");
        this->motor.set_speed(MotorB, Backward, DEFAULT_MOTOR_SPEED);
    } else {
        // Serial.println("rotate right");
        // rotate right
        this->motor.set_speed(MotorA, Forward, DEFAULT_MOTOR_SPEED);
    }
    // -5 <= roverAngle(266) <= 5
    Serial.println("rotating...");
    while (outOfRange(validRange, 7, this->roverAngle)) {
        // Serial.println("rotating...");
        vTaskDelay(70);
    }
    this->stopMotors();
    Serial.println("rotate end");
}


void logic::setSensor(sclass sensor) {
    Serial.println("setSensor");
    this->sensor = sensor;
}

