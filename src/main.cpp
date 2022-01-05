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

void messageHandler(String &topic, String &payload) {
    Serial.println("incoming: " + topic + " - " + payload);
    if (topic.equals("greengrass/group10rover")) roverLogic.updateRover(payload);
    else roverLogic.updateTarget(payload);
}

void taskOne( void * parameter )
{
  Serial.println("AWS taskOne");
  myawsclass awsobject = myawsclass();
  awsobject.connectAWS();
  awsobject.messageHandler(messageHandler);
  //example of a task that executes for some time and then is deleted
  for( int i = 0; i < 1500; i++ ) {
      bool b = awsobject.stayConnected();
      if (!b) awsobject.connectAWS();
      
      vTaskDelay(1000 / portTICK_PERIOD_MS); //this pauses the task, so others can execute
    
  }
  Serial.println("Ending task: 1"); //should not reach this point.
  vTaskDelete( NULL );
}
 
void taskTwo( void * parameter)
{
  sclass sensorobject = sclass();
  sensorobject.SETUP();
  roverLogic.setSensor(sensorobject);

  mclass motorobject = mclass();
  motorobject.SETUP();
  roverLogic.setMotor(motorobject);

  roverLogic.reachTarget();

    // create an endless loop so the task executes forever
    for( ;; )
    {
      int16_t *arr =  sensorobject.reading();
        Serial.println("Hello from task: 2");
        for(int i = 0; i < 3; i++)
          Serial.println(arr[i]);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    Serial.println("Ending task 2"); //should not reach this point.
    vTaskDelete( NULL );
}
