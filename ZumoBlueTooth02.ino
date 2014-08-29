#include <ZumoBuzzer.h>
#include <stdlib.h>
#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <Wire.h>
#include <LSM303.h>

#define THROTTLE_PIN   4 // throttle channel from RC receiver
#define STEERING_PIN   5 // steering channel from RC receiver
#define LED_PIN       13 // user LED pin

#define MAX_SPEED             400 // max motor speed
#define PULSE_WIDTH_DEADBAND   25 // pulse width difference from 1500 us (microseconds) to ignore (to compensate for control centering offset)
#define PULSE_WIDTH_RANGE     350 // pulse width difference from 1500 us to be treated as full scale input (for example, a value of 350 means
//   any pulse width <= 1150 us or >= 1850 us is considered full scale)

#define QTR_THRESHOLD  1000 // microseconds                                  
// Accelerometer Settings
#define RA_SIZE 3  // number of readings to include in running average of accelerometer readings
#define XY_ACCELERATION_THRESHOLD 2400  // for detection of contact (~16000 = magnitude of acceleration due to gravity)


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

unsigned long loop_start_time;
unsigned long last_turn_time;
unsigned long contact_made_time;
#define MIN_DELAY_AFTER_TURN          400  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   1000  // ms = min delay between detecting new contact event

template <typename T> 
class RunningAverage
{
public:
  RunningAverage(void);
  RunningAverage(int);
  ~RunningAverage();
  void clear();
  void addValue(T);
  T getAverage() const;
  void fillValue(T, int);
protected:
  int _size;
  int _cnt;
  int _idx;
  T _sum;
  T * _ar;
  static T zero;
};

class Accelerometer : 
public LSM303
{
  typedef struct acc_data_xy
  {
    unsigned long timestamp;
    int x;
    int y;
    float dir;
  } 
  acc_data_xy;

public: 
  Accelerometer() : 
  ra_x(RA_SIZE), ra_y(RA_SIZE) {
  };
  ~Accelerometer() {
  };
  void enable(void);
  void readAcceleration(unsigned long timestamp);
  float len_xy() const;
  float dir_xy() const;
  int x_avg(void) const;
  int y_avg(void) const;
  long ss_xy_avg(void) const;
  float dir_xy_avg(void) const;
private:
  acc_data_xy last;
  RunningAverage<int> ra_x;
  RunningAverage<int> ra_y;   
};

Accelerometer lsm303;
boolean in_contact;  // set when accelerometer detects contact with opposing robot

unsigned long time;

ZumoMotors motors;
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define SEARCH_SPEED      200
#define SUSTAINED_SPEED   400 // switches to SUSTAINED_SPEED from FULL_SPEED after FULL_SPEED_DURATION_LIMIT ms
#define FULL_SPEED        400
#define STOP_DURATION     100 // ms
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms
#define RIGHT 1
#define LEFT -1
enum ForwardSpeed { SearchSpeed, SustainedSpeed, FullSpeed };
ForwardSpeed _forwardSpeed;  // current forward speed setting
unsigned long full_speed_start_time;
#define FULL_SPEED_DURATION_LIMIT     250  // ms

ZumoBuzzer buzzer;
const char sound_effect[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody


void setup()
{
  pinMode(13,OUTPUT);
  Serial.begin(38400); //AT+UART=115200  AT+UART=38400
  digitalWrite(13,LOW);
  Wire.begin();
  lsm303.init();
  lsm303.enable();
  time = millis();
}

void on_contact_made()
{
  in_contact = true;
  contact_made_time = loop_start_time;
  motors.setLeftSpeed(0);  
  motors.setRightSpeed(0);
  buzzer.playFromProgramSpace(sound_effect);
}

bool check_for_contact()
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return (lsm303.ss_xy_avg() >  threshold_squared) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
}

void loop()
{
  //   return;
  sensors.read(sensor_values);
  lsm303.readAcceleration(loop_start_time); 
  if (check_for_contact()) {
    on_contact_made();
  }
  if ((sensor_values[0] > QTR_THRESHOLD) or (sensor_values[5] > QTR_THRESHOLD)) {
    motors.setLeftSpeed(0);  
    motors.setRightSpeed(0);
    Serial.print("z}"); 
    motors.setLeftSpeed(-200);  
    motors.setRightSpeed(-200);
    delay(150);
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
      case LEFT_FORWARD:    
        motors.setLeftSpeed(200);   
        break;  //Serial.print(re); Serial.print('}');
      case LEFT_BACKWARD:   
        motors.setLeftSpeed(-200);  
        break;   
      case LEFT_STOP:       
        motors.setLeftSpeed(0);  
        break; 
      case RIGHT_FORWARD:   
        motors.setRightSpeed(200); 
        break;  
      case RIGHT_BACKWARD:  
        motors.setRightSpeed(-200);   
        break;   
      case RIGHT_STOP:      
        motors.setRightSpeed(0); 
        break; 
      case BOTH_FORWARD:    
        motors.setLeftSpeed(200);  
        motors.setRightSpeed(200); 
        break; 
      case BOTH_BACKWARD:   
        motors.setLeftSpeed(-200);  
        motors.setRightSpeed(-200);  
        break;  
      case BOTH_BOTH:       
        motors.setLeftSpeed(0);  
        motors.setRightSpeed(0);    
        break;  
        //case SERVO_FORWARD:   topServoPos = topServoPos + 5; setTopServoPos(topServoPos);
        //case SERVO_BACKWARD:  topServoPos = topServoPos - 5; setTopServoPos(topServoPos);
      }  

    }
  }   
  if ((millis()-time) > 250) {
    time = millis();
    Serial.print(sensor_values[0]); 
    Serial.print(","); 
    Serial.print(sensor_values[1]); 
    Serial.print(","); 
    Serial.print(sensor_values[2]); 
    Serial.print(","); 
    Serial.print(sensor_values[3]); 
    Serial.print(","); 
    Serial.print(sensor_values[4]); 
    Serial.print(","); 
    Serial.print(sensor_values[5]); 
    Serial.print("}"); 
  }   

}

void Accelerometer::enable(void)
{
  // Enable Accelerometer
  // 0x27 = 0b00100111
  // Normal power mode, all axes enabled
  writeAccReg(LSM303::CTRL_REG1_A, 0x27);

  if (getDeviceType() == LSM303::device_DLHC)
    writeAccReg(LSM303::CTRL_REG4_A, 0x08); // DLHC: enable high resolution mode
}


void Accelerometer::readAcceleration(unsigned long timestamp)
{
  readAcc();
  if (a.x == last.x && a.y == last.y) return;

  last.timestamp = timestamp;
  last.x = a.x;
  last.y = a.y;

  ra_x.addValue(last.x);
  ra_y.addValue(last.y);
}

float Accelerometer::len_xy() const
{
  return sqrt(last.x*a.x + last.y*a.y);
}

float Accelerometer::dir_xy() const
{
  return atan2(last.x, last.y) * 180.0 / M_PI;
}

int Accelerometer::x_avg(void) const
{
  return ra_x.getAverage();
}

int Accelerometer::y_avg(void) const
{
  return ra_y.getAverage();
}

long Accelerometer::ss_xy_avg(void) const
{
  long x_avg_long = static_cast<long>(x_avg());
  long y_avg_long = static_cast<long>(y_avg()); 
  return x_avg_long*x_avg_long + y_avg_long*y_avg_long;
}

float Accelerometer::dir_xy_avg(void) const
{
  return atan2(static_cast<float>(x_avg()), static_cast<float>(y_avg())) * 180.0 / M_PI;
}



// RunningAverage class 
// based on RunningAverage library for Arduino
// source:  http://playground.arduino.cc/Main/RunningAverage
// author:  Rob.Tillart@gmail.com
// Released to the public domain

template <typename T>
T RunningAverage<T>::zero = static_cast<T>(0);

template <typename T>
RunningAverage<T>::RunningAverage(int n)
{
  _size = n;
  _ar = (T*) malloc(_size * sizeof(T));
  clear();
}

template <typename T>
RunningAverage<T>::~RunningAverage()
{
  free(_ar);
}

// resets all counters
template <typename T>
void RunningAverage<T>::clear() 
{ 
  _cnt = 0;
  _idx = 0;
  _sum = zero;
  for (int i = 0; i< _size; i++) _ar[i] = zero;  // needed to keep addValue simple
}

// adds a new value to the data-set
template <typename T>
void RunningAverage<T>::addValue(T f)
{
  _sum -= _ar[_idx];
  _ar[_idx] = f;
  _sum += _ar[_idx];
  _idx++;
  if (_idx == _size) _idx = 0;  // faster than %
  if (_cnt < _size) _cnt++;
}

// returns the average of the data-set added so far
template <typename T>
T RunningAverage<T>::getAverage() const
{
  if (_cnt == 0) return zero; // NaN ?  math.h
  return _sum / _cnt;
}

// fill the average with a value
// the param number determines how often value is added (weight)
// number should preferably be between 1 and size
template <typename T>
void RunningAverage<T>::fillValue(T value, int number)
{
  clear();
  for (int i = 0; i < number; i++) 
  {
    addValue(value);
  }
}



