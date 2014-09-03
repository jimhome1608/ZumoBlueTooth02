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



const char LED_OFF =        'A';
const char LED_ON  =        'B';
const char LEFT_FORWARD  =  'C';
const char LEFT_BACKWARD =  'D';
const char LEFT_STOP  =     'E';
const char RIGHT_FORWARD  = 'F';
const char RIGHT_BACKWARD = 'G';
const char RIGHT_STOP  =    'H';
const char BOTH_FORWARD  =  'I';
const char BOTH_BACKWARD  = 'J';
const char BOTH_BOTH  =     'K';
const char GO_LEFT  =       'L';
const char GO_RIGHT  =      'M';
//-------------------------------

unsigned long loop_start_time;
unsigned long last_sent_readings;
unsigned long last_motor_change = 0;
int speedLeft = 0;
int speedRight = 0;

double _runningSpeed = 0;

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
    int z;
    float dir;
  } 
  acc_data_xy;

public: 
  Accelerometer() : 
  ra_x(RA_SIZE), ra_y(RA_SIZE), ra_z(RA_SIZE) {
  };
  ~Accelerometer() {
  };
  void enable(void);
  void readAcceleration(unsigned long timestamp);
  float len_xy() const;
  float dir_xy() const;
  int x_avg(void) const;
  int y_avg(void) const;
  int z_avg(void) const;
  long ss_xy_avg(void) const;
  float dir_xy_avg(void) const;
private:
  acc_data_xy last;
  RunningAverage<int> ra_x;
  RunningAverage<int> ra_y; 
  RunningAverage<int> ra_z;   
};

Accelerometer lsm303;
boolean in_contact;  // set when accelerometer detects contact with opposing robot



ZumoMotors motors;
#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

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
  last_sent_readings = 0;
}


bool check_for_contact()
{
  int XY_ACCELERATION_THRESHOLD = 3000;  // 2400 for detection of contact (~16000 = magnitude of acceleration due to gravity)
  if ( (speedRight == 0) && (speedLeft == 0) ) 
    return(0);  
  long int _avg = 0;
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  _avg = lsm303.ss_xy_avg();
  bool _result=  (_avg >  threshold_squared);
  if ((millis() -last_motor_change)  < 400)
    _result = 0;
  if (_result) {
    setLeftMotorSpeed(0);  
    setRightMotorSpeed(0);
    last_motor_change = millis();
    Serial.print("z Hit}");     
    buzzer.playNote(NOTE_C(1), 200, 15);
    delay(1000);
  }  
  return(_result);
}

bool check_tilt()
{
  if ( (speedRight == 0) && (speedLeft == 0) ) 
    return(0);  
  bool _result= (abs(lsm303.z_avg()) > 1000);   
  if ((millis() -last_motor_change)  < 400)
    _result = 0;
  if (_result) {
    setLeftMotorSpeed(0);  
    setRightMotorSpeed(0);
    last_motor_change = millis();
    Serial.print("z Tilt}");     
    buzzer.playNote(NOTE_C(2), 200, 15);
    delay(1000);
  }    
  return(_result);
}

void setLeftMotorSpeed(double  _perecent)
{
  int _speed = floor(400 * _perecent);
  motors.setLeftSpeed(_speed);
  if (_speed != speedLeft)
     last_motor_change = millis(); 
  speedLeft = _speed;
}  


void setRightMotorSpeed(double  _perecent)
{
 int _speed = floor(400 * _perecent); 
  motors.setRightSpeed(_speed);
  if (_speed != speedRight)
     last_motor_change = millis(); 
  speedRight = _speed;
} 

void goLeft()
{
  int _delay = 30;
  for (int i=0;i<5;i++) {
     setRightMotorSpeed(0.75);  
     delay(_delay);
     setRightMotorSpeed(0);  
     setLeftMotorSpeed(0.75);
     delay(_delay);
     setLeftMotorSpeed(0); 
     setRightMotorSpeed(-0.75);  
     delay(_delay);
     setRightMotorSpeed(0);  
     setLeftMotorSpeed(-0.75);
     delay(_delay);
     setLeftMotorSpeed(0);
 }
}

void goRight()
{
  int _delay = 30;
  for (int i=0;i<5;i++) {
     //setLeftMotorSpeed(THREE_QUARTER_SPEED); 
     setLeftMotorSpeed(0.75);
     delay(_delay);
     setLeftMotorSpeed(0);  
     setRightMotorSpeed(0.75);
     delay(_delay);
     setRightMotorSpeed(0); 
     setLeftMotorSpeed(-0.75);
     delay(_delay);
     setLeftMotorSpeed(0);  
     setRightMotorSpeed(-0.75);
     delay(_delay);
     setRightMotorSpeed(0);
 }
}

void loop()
{  
  
  sensors.read(sensor_values);
  lsm303.readAcceleration(loop_start_time); 
  boolean _RunningForward =  (speedRight > 0) && (speedLeft > 0);
  boolean _RunningBackward =  (speedRight < 0) && (speedLeft < 0);
  if ( _RunningForward || _RunningBackward ) {
     check_for_contact();
  }
  //check_tilt();
  //return;
  if ( (speedRight > 0) || (speedLeft > 0) ) {
      if ((sensor_values[0] > QTR_THRESHOLD) or (sensor_values[5] > QTR_THRESHOLD)) {
        setLeftMotorSpeed(0);  
        setRightMotorSpeed(0);
         Serial.print("z Edge}"); 
        buzzer.playNote(NOTE_C(3), 200, 15);
        delay(1000);
        /*setLeftMotorSpeed(-1.0);  
        setRightMotorSpeed(-1.0);
        delay(150);
        setLeftMotorSpeed(0);  
        setRightMotorSpeed(0);  */    
      };  
  };
  char re = ' ';
  char dat= ' ';
  
  if (Serial.available()>0) {
    while (Serial.available()>0) {
      re = Serial.read();
      if (Serial.available()>0) {
        dat = Serial.read();
        _runningSpeed = (dat-48);
        if (_runningSpeed ==0)
           _runningSpeed = 1;
        else
        {
          _runningSpeed = _runningSpeed/10;
        }  
      }
      switch(re) {
      case LED_OFF: 
        digitalWrite(13,LOW); 
        break;  
      case LED_ON: 
        digitalWrite(13,HIGH); 
        break;  
      case LEFT_FORWARD:   
        setLeftMotorSpeed(_runningSpeed);   
        break;  
      case LEFT_BACKWARD:   
        setLeftMotorSpeed(-_runningSpeed);  
        break;   
      case LEFT_STOP:       
        setLeftMotorSpeed(0);  
        break; 
      case RIGHT_FORWARD:   
        setRightMotorSpeed(_runningSpeed); 
        break;  
      case RIGHT_BACKWARD:  
        setRightMotorSpeed(-_runningSpeed);   
        break;   
      case RIGHT_STOP:      
        setRightMotorSpeed(0); 
        break; 
      case BOTH_FORWARD:    
        setLeftMotorSpeed(_runningSpeed);  
        setRightMotorSpeed(_runningSpeed); 
        break; 
      case BOTH_BACKWARD:   
        setLeftMotorSpeed(-_runningSpeed);  
        setRightMotorSpeed(-_runningSpeed);  
        break;  
      case BOTH_BOTH:       
        setLeftMotorSpeed(0);  
        setRightMotorSpeed(0);    
        break;  
      case GO_LEFT:   
        goLeft();
        break; 
      case GO_RIGHT:  
        goRight();
        break; 
      }  
    }
  }  
  if ((millis()-last_sent_readings) > 250) {
    last_sent_readings = millis();
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
    Serial.println("}"); 
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
  last.z = a.z;

  ra_x.addValue(last.x);
  ra_y.addValue(last.y);
  ra_z.addValue(last.y);
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

int Accelerometer::z_avg(void) const
{
  return ra_z.getAverage();
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



