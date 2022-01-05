#include "logic.h"
#include <ArduinoJson.h>

logic::logic() {
    this->targetX = -1;
    this-> targetY = -1;
    this-> roverX = -1;
    this->roverY = -1;
    this->roverAngle = -1;
}

void logic::roverReadyToMove() {
    while (this->targetX == -1 || this->targetY == -1 ||
        this->roverX == -1 || this->roverY == -1 || this->roverAngle == -1 ||
        &this->motor == nullptr || &this->sensor == nullptr) {
        delay(100);
    }
    
}

void logic::reachTarget() {
    this->roverReadyToMove();
}

void logic::updateRover(String &payload) {
    // {"21": [[126, 43], 276]}
    Serial.println("updateRover");
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    int x = doc["21"][0][0];
    Serial.println(x);
    x = doc["21"][0][1];
    Serial.println(x);
    x = doc["21"][1];
    Serial.println(x);
}

void logic::updateTarget(String &payload) {
    // [405, 293]
    Serial.println("updateTarget");
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    int x = doc[0];
    Serial.println(x);
    x = doc[1];
    Serial.println(x);
}

void logic::setMotor(mclass motor) {
    Serial.println("setMotor");
    this->motor = motor;
    this->motor.set_speed(MotorA, Forward, 0);
    this->motor.set_speed(MotorB, Backward, 0);
}

void logic::setSensor(sclass sensor) {
    Serial.println("setSensor");
    this->sensor = sensor;
}

