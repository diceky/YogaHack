#include "CurieIMU.h"
#include <MadgwickAHRS.h>

Madgwick filter; 
int factor = 800;

#define NUM 1
#define WAITCOUNT 5
#define THRESHCOUNT 15

int b_pin = 7;
int b_led = 2;
int alert_led = 4;
int state = 0;
static int switchFlag = 0;
static int fixedBPM;
static int breathFlag= 0;// 0: INHALE START, 1: INHALE END, 2: EXHALE START, 3: EXHALE END
float timer2 = 0, timer3 = 0;

int temp[WAITCOUNT];
int count = 0;

int thresh_count = 0;
int max = 0, min = 1000, thresh = 0, thresh_flag = 0;

static int detect_flag = 0;
int breath = 0;
static int compare_before = 0, compare_current = 0;

float bpmNow=0, bpmBefore=0;
float lastBreathTime=0, currentBreathTime=0;
 
void setup() {

  Serial.begin(115200);
  pinMode(b_pin, INPUT);
  pinMode(b_led, OUTPUT);
  pinMode(alert_led, OUTPUT);

  // BUILT IN ACCELEROMETER SETUP
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);
  
  for(int i = 0; i < WAITCOUNT; i++){
   temp[i] = 0; 
  }
}
 
void loop() {
  
  //ACCELEROMETER STUFF
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);  
  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);
  // update the filter, which computes orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  /*
  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);
  */

  // READ SWITCH STATE
  state = digitalRead(b_pin);  // state of tact switch
  //Serial.println(state);
  if(state == 0){
    switchFlag = 1;
    Serial.println("Switch Flag ON");
  }

  //GET RUBBER SENSOR VALUE
  int ain = analogRead(A0);
  //Serial.println(ain);

  //CALCULATE AVERAGE
  if(count <  WAITCOUNT){   //COLLECT DATA FOR AVERAGE
    temp[count] = ain;
    count++;
  }
  else{   //READY TO CALCULATE AVERAGE
   int average = 0;
   for(int i = 0; i < WAITCOUNT; i++){
    average += temp[i];
    temp[i] = 0;
   }
   average = average / WAITCOUNT;
   count = 0;
   float timer = millis();
   Serial.print(timer / 1000);
   Serial.print(",AVERAGE: ");
   Serial.println(average);
  
   //CALCULATE THRESHOLD
   if(thresh_count < THRESHCOUNT){
     if(average > max) max = average;
     else if(average < min) min = average;
     thresh_count ++;
     //Serial.println("MAX: " + max + " MIN: " + min);
    }
   else{
    if(detect_flag == 0) detect_flag = 1;   //DETECTED FIRST THRESHOLD
    thresh = (max+min) / 2;
    max = 0;
    min = 1000;
    thresh_count = 0;
   }
   //Serial.println("COUNT: " + thresh_count + " THRESH: " + thresh);
  
  //DETECT BREATH
  if(detect_flag == 1){
    compare_before = compare_current;
    compare_current = average;
    //Serial.println("comparing");
    if(compare_before > thresh && compare_current <= thresh){
      breath++;
      Serial.print(timer / 1000);
      Serial.println(",BREATH_DETECTED");
      digitalWrite(b_led, HIGH);    //blink LED
      delay(100);
      lastBreathTime = currentBreathTime;
      currentBreathTime = timer;
      bpmBefore = bpmNow;
      bpmNow = 60 / ((currentBreathTime - lastBreathTime) / 1000);
      bpmNow = (bpmBefore + bpmNow) / 2;
      Serial.print("BPM: ");
      Serial.println(bpmNow);
    }
    digitalWrite(b_led, LOW);
  }

  if(switchFlag == 1){
    fixedBPM = bpmNow;
    Serial.println("BPM FIXED");
    switchFlag = 0;
  }

  if(fixedBPM > 0){
    if(abs(fixedBPM - bpmNow) > 6){   //if current BPM differs greatly from fixed BPM
      digitalWrite(alert_led, HIGH);  //blink alert LED
      delay(100);
      float duration = ((float)60 / (float)fixedBPM) / (float)2 * (float)1000;
      Serial.print("fixedBPM: ");
      Serial.println(fixedBPM);
      Serial.print("Duration: ");
      Serial.println(duration);
      if(breathFlag == 0){
        timer2 = millis();  //breath start time set
        breathFlag = 1;
        analogWrite(9, 0);
      }
      else if(breathFlag==1){
        if (millis() - timer2 < duration) {
        }
        else{
          breathFlag = 2;
          analogWrite(9, 150);  //short vibration in between breath
        }
        //Serial.println(millis() - timer2);
      }
      else if(breathFlag == 2){
        timer3 = millis();  //breath end time set
        breathFlag = 3;
        analogWrite(9, 0);
      }
      else if(breathFlag==3){
        if (millis() - timer3 < duration) {
        }
        else{
          breathFlag = 0;
          analogWrite(9, 150);  //short vibration in between breath
        }
        //Serial.println(millis() - timer3);
      }
      Serial.print("BREATH FLAG: ");
      Serial.println(breathFlag);
    }//close if(abs(fixedBPM - bpmNow) > 3)
    else analogWrite(9, 0);
    digitalWrite(alert_led, LOW);
   }//close if fixedBPM>0
  }//close else //READY TO CALCULATE AVERAGE
  
  delay(100);
}


float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

