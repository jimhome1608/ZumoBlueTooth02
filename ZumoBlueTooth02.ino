 
#include <stdlib.h>
#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>

#define THROTTLE_PIN   4 // throttle channel from RC receiver
#define STEERING_PIN   5 // steering channel from RC receiver
#define LED_PIN       13 // user LED pin

#define MAX_SPEED             400 // max motor speed
#define PULSE_WIDTH_DEADBAND   25 // pulse width difference from 1500 us (microseconds) to ignore (to compensate for control centering offset)
#define PULSE_WIDTH_RANGE     350 // pulse width difference from 1500 us to be treated as full scale input (for example, a value of 350 means
                                  //   any pulse width <= 1150 us or >= 1850 us is considered full scale)
                                  
#define QTR_THRESHOLD  1000 // microseconds                                  


const char LED_OFF =        '0';
const char LED_ON  =        '1';
const char LEFT_FORWARD  =  '2';
const char LEFT_BACKWARD =  '3';
const char LEFT_STOP  =     '4';
const char RIGHT_FORWARD  = '5';
const char RIGHT_BACKWARD = '6';
const char RIGHT_STOP  =    '7';
const char BOTH_FORWARD  =  '8';
const char BOTH_BACKWARD  = '9';
const char BOTH_BOTH  =     'A';
const char SERVO_FORWARD  =      'B';
const char SERVO_BACKWARD  =     'C';
//-------------------------------

unsigned long time;

ZumoMotors motors;
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

void setup()
{
  pinMode(13,OUTPUT);
  Serial.begin(38400); //AT+UART=115200  AT+UART=38400
  digitalWrite(13,LOW);
  time = millis();
}


void loop()
{
//   return;
  sensors.read(sensor_values);
  if (sensor_values[0] > QTR_THRESHOLD) {
     motors.setLeftSpeed(0);  
     motors.setRightSpeed(0);
  };  
  if (sensor_values[5] > QTR_THRESHOLD){
     motors.setLeftSpeed(0);  
     motors.setRightSpeed(0);
  };  
  char re;
  char buf[7];
  re = ' ';
  if (Serial.available()>0) {
    while (Serial.available()>0) {
      re = Serial.read();
      switch(re) {
          case LED_OFF: 
            digitalWrite(13,LOW); 
            Serial.print(re); 
            Serial.print('}');
            break;  
          case LED_ON: 
            digitalWrite(13,HIGH); 
            Serial.print(re); 
            Serial.print('}');
            break;  
          case LEFT_FORWARD:    motors.setLeftSpeed(200);   break;  //Serial.print(re); Serial.print('}');
          case LEFT_BACKWARD:   motors.setLeftSpeed(-200);  break;   
          case LEFT_STOP:       motors.setLeftSpeed(0);  break; 
          case RIGHT_FORWARD:   motors.setRightSpeed(200); break;  
          case RIGHT_BACKWARD:  motors.setRightSpeed(-200);   break;   
          case RIGHT_STOP:      motors.setRightSpeed(0); break; 
          case BOTH_FORWARD:    motors.setLeftSpeed(200);  motors.setRightSpeed(200); break; 
          case BOTH_BACKWARD:   motors.setLeftSpeed(-200);  motors.setRightSpeed(-200);;  break;  
          case BOTH_BOTH:       motors.setLeftSpeed(0);  motors.setRightSpeed(0);    break;  
          //case SERVO_FORWARD:   topServoPos = topServoPos + 5; setTopServoPos(topServoPos);
          //case SERVO_BACKWARD:  topServoPos = topServoPos - 5; setTopServoPos(topServoPos);
     }  
      
    }
  }   
  if ((millis()-time) > 100) {
     time = millis();
     Serial.print(sensor_values[0]); 
     Serial.println("}"); 
     Serial.print(sensor_values[5]); 
     Serial.println("}"); 
  }   
}


