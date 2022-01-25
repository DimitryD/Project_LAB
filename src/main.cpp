#include <Arduino.h>
#include "AWS.h"
#include "sensor.h"
#include "sensorDriver.h"
#include "motorDriver.h"
#include "logic.h"

void taskOne( void * parameter);
void taskTwo( void * parameter);
#define LED_BOARD 2 //change here the pin of the board to V2

logic roverLogic = logic();

void setup(){
  pinMode(LED_BOARD, OUTPUT);
  Serial.begin(9600);
  delay(500);
  // task for AWS
  xTaskCreate(
                    taskOne,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    9048,              /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */

  // task for logic
  xTaskCreate(
                    taskTwo,          /* Task function. */
                    "TaskTwo",        /* String with name of task. */
                    9048,              /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
}

void loop(){
  delay(1000);
}

// this is a callback function to process the received message.
void messageHandler(String &topic, String &payload) {
    // Serial.println("incoming: " + topic + " - " + payload);
    if (&payload == nullptr || payload.equals("null") || payload == NULL) return;    
    // if the topic is the rover then updateRover with new payload 
    //(message received from AWS as follows {"21": [[126, 43], 276]})
    if (topic.equals("greengrass/group10rover")) roverLogic.updateRover(payload);
    else roverLogic.updateTarget(payload);
}

// ------------------- TASK ONE responsible for keeping aws connected  ----------------------------
void taskOne( void * parameter )
{
  Serial.println("AWS taskOne");
  myawsclass awsobject = myawsclass();
  // we connect to wifi and AWS
  awsobject.connectAWS();
  // to receive messages we need to create a callback function
  awsobject.messageHandler(messageHandler);

  //example of a task that executes for some time and then is deleted
  for(;; ) {
      bool b = awsobject.stayConnected();
      if (!b) awsobject.connectAWS();
      vTaskDelay(5); //this pauses the task, so others can execute
  }
  Serial.println("Ending task: 1"); //should not reach this point.
  vTaskDelete( NULL );
}
 

// ------------------- TASK TWO used to setup sensor, motors and calling method to reach target  ----------------------------
void taskTwo(void * parameter) {
  Serial.println("Logic taskTwo");

  Serial.println("init sensor");
  sclass sensorobject = sclass();
  // setup sensor object
  sensorobject.SETUP();
  // set sensor values
  roverLogic.setSensor(sensorobject);

  Serial.println("init motor");
  mclass motorobject = mclass();
  // setup motor object
  motorobject.SETUP();
  //set motor object and mobe it forward
  roverLogic.setMotor(motorobject);

  //main function to reach target while avoiding obstacles
  roverLogic.reachTarget();

   Serial.println("Ending task 2"); //should not reach this point.
  vTaskDelete( NULL );
}
