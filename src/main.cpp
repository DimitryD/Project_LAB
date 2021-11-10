#include <Arduino.h>
#include "AWS.h"
#include "sensor.h"
#include "sensorDriver.h"
#include "motorDriver.h"

void taskOne( void * parameter);
void taskTwo( void * parameter);
#define LED_BOARD 2 //change here the pin of the board to V2

void setup(){
  pinMode(LED_BOARD, OUTPUT);
  Serial.begin(9600);
  delay(500);
  xTaskCreate(
                    taskOne,          /* Task function. */
                    "TaskOne",        /* String with name of task. */
                    2048,              /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */

  xTaskCreate(
                    taskTwo,          /* Task function. */
                    "TaskTwo",        /* String with name of task. */
                    2048,              /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    1,                /* Priority of the task. */
                    NULL);            /* Task handle. */
}

void loop(){
delay(1000);
}

void taskOne( void * parameter )
{
  //   myawsclass awsobject = myawsclass();
  // awsobject.connectAWS();
  //     Serial.println("connect AWS successsssssssssssss");

  // awsobject.stayConnected();
  //       Serial.println("stayconnect AWS successsssssssssssss");

    //example of a task that executes for some time and then is deleted
    // for( int i = 0; i < 500; i++ )
    // {
    //   Serial.println("Hello from task 1");
      
    //   //Switch on the LED
    //   digitalWrite(LED_BOARD, HIGH); 
    //   // Pause the task for 1000ms
    //   //delay(1000); //This delay doesn't give a chance to the other tasks to execute
    //   vTaskDelay(100 / portTICK_PERIOD_MS); //this pauses the task, so others can execute
    //   // Switch off the LED
    //   digitalWrite(LED_BOARD, LOW);
    //   // Pause the task again for 500ms
    //   vTaskDelay(100 / portTICK_PERIOD_MS);
    // }
    Serial.println("Ending task: 1");
    vTaskDelete( NULL );
}
 
void taskTwo( void * parameter)
{
  sclass sensorobject = sclass();
  sensorobject.SETUP();
  mclass motorobject = mclass();
  motorobject.SETUP();
  motorobject.set_speed(MotorA, Forward, 0);
  motorobject.set_speed(MotorB, Backward, 0);
    //create an endless loop so the task executes forever
    for( ;; )
    {
      int16_t *arr =  sensorobject.reading();
        Serial.println("Hello from task: 2");
        for(int i = 0; i < 3; i++)
          Serial.println(arr[i]);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    Serial.println("Ending task 2"); //should not reach this point but in case...
    vTaskDelete( NULL );
}
