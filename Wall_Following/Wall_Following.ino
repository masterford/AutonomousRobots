#include <Servo.h>

#define servoPin 7 // pin for servo signal

#define motorFwdPin 8 // HIGH for FWD; LOW for REV
#define motorRevPin 9 // LOW for FWD; HIGH for REV
#define motorLPWMPin 10 // Left Motor Speed Control
#define motorRPWMPin 11 // Right Motor Speed Control

#define pingTrigPin0 23 // left ping sensor trigger pin (output from Arduino)
#define pingEchoPin0 25 // left ping sensor echo pin (input to Arduino)
#define pingGrndPin0 27 // left ping sensor ground pin (use digital pin as ground)

#define pingTrigPin1 22 // center ping sensor trigger pin (output from Arduino)
#define pingEchoPin1 24 // center ping sensor echo pin (input to Arduino)
#define pingGrndPin1 26 // center ping sensor ground pin (use digital pin as ground)

#define pingTrigPin2 6  // right ping sensor trigger pin (output to Arduino)
#define pingEchoPin2 5  // right ping sensor echo pin (input to Arduino

#define KI 0.1
#define KP 0.5
#define distanceCenter 30

//
// global variables
// (yucky but needed to make i2c interrupts work later)
//
float rightPingDistanceCM = 0.0;
float centerPingDistanceCM = 0.0;
float leftPingDistanceCM = 0.0;

Servo steeringServo;
double servoAngleDeg = -10.0;

byte motorLPWM=100;
byte motorRPWM=250;

static int integratedRightPingDistanceCM = 0;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("MegaPingTest");

      //left ping sensor set-up
    pinMode(pingGrndPin0,OUTPUT); digitalWrite(pingGrndPin0,LOW);
    pinMode(pingTrigPin0,OUTPUT);
    pinMode(pingEchoPin0,INPUT);

  //center ping sensor set-up
    pinMode(pingGrndPin1,OUTPUT); digitalWrite(pingGrndPin1,LOW);
    pinMode(pingTrigPin1,OUTPUT);
    pinMode(pingEchoPin1,INPUT);

  //right ping sensor set-up
    pinMode(pingTrigPin2,OUTPUT);
    pinMode(pingEchoPin2,INPUT);

  //Motor set-up
    pinMode(motorFwdPin,OUTPUT); digitalWrite(motorFwdPin,LOW);
    pinMode(motorRevPin,OUTPUT); digitalWrite(motorRevPin,HIGH);
    pinMode(motorLPWMPin,OUTPUT); analogWrite(motorLPWMPin,motorLPWM);
    pinMode(motorRPWMPin,OUTPUT); analogWrite(motorRPWMPin,motorRPWM);

  //Servo set-up
    steeringServo.attach(servoPin);
    setServoAngle(servoAngleDeg);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  static unsigned long timeLast = 0;

  getPingDistanceCM(1);

  if(centerPingDistanceCM <= 30){
    analogWrite(motorLPWMPin,0);
    analogWrite(motorRPWMPin,0);
  }else{

    analogWrite(motorLPWMPin,motorLPWM);
    analogWrite(motorRPWMPin,motorRPWM);
  }
    
  // Wall follow on right side (for now)
  getPingDistanceCM(2);
  /*Serial.print("Right Ping Distance: ");
  Serial.print(rightPingDistanceCM, DEC);
  Serial.print(" Integrated Right  Ping: ");
  Serial.println(integratedRightPingDistanceCM, DEC);
  */

  float dt = micros() - timeLast;
  timeLast = micros();
  float err = rightPingDistanceCM - distanceCenter;
  static float deltaI = 0;
  deltaI += KI * dt * err * 0.001 * 0.001;
  deltaI = constrain(deltaI, -20, 20);
  float servoDeg = deltaI + KP * err;
  setServoAngle(servoDeg);

  Serial.print("deltaI: ");
  Serial.print(deltaI, DEC);
  Serial.print("Right Ping: ");
  Serial.print(rightPingDistanceCM, DEC);
  Serial.print(" Servo Deg: ");
  Serial.println(servoDeg, DEC);

}


void getPingDistanceCM(int rcl){
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = 3000;
  //
  // pingTrigPin0 = trigger pin
  // pingEchoPin0 = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  if(rcl==0){ //left
    digitalWrite(pingTrigPin0, LOW);
    delayMicroseconds(2);
    digitalWrite(pingTrigPin0, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingTrigPin0, LOW);
    //
    // The echo pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    //
    unsigned long echo_time_l;
    echo_time_l = pulseIn(pingEchoPin0, HIGH, timeout_us);
    if (echo_time_l == 0){
      echo_time_l = timeout_us;
    }
    //
    // return the distance in centimeters
    // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
    // divide by 2 because we measure "round trip" time for echo
    // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
    // = 0.017*echo_time
    //
    leftPingDistanceCM = constrain(0.017*echo_time_l,5.0,50.0);
  } 
  else if (rcl==1) { //center
    digitalWrite(pingTrigPin1, LOW);
    delayMicroseconds(2);
    digitalWrite(pingTrigPin1, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingTrigPin1, LOW);
    //
    // The echo pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    //
    unsigned long echo_time_c;
    echo_time_c = pulseIn(pingEchoPin1, HIGH, timeout_us);
    if (echo_time_c == 0){
      echo_time_c = timeout_us;
   }
    //
    // return the distance in centimeters
    // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
    // divide by 2 because we measure "round trip" time for echo
    // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
    // = 0.017*echo_time
    //
    centerPingDistanceCM = constrain(0.017*echo_time_c,5.0,50.0);
  } 
  else if(rcl==2){ //right
    
    digitalWrite(pingTrigPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(pingTrigPin2, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingTrigPin2, LOW);
    //
    // The echo pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    //
    unsigned long echo_time_r;
    echo_time_r = pulseIn(pingEchoPin2, HIGH, timeout_us);
    if (echo_time_r == 0){
      echo_time_r = timeout_us;
   }
    //
    // return the distance in centimeters
    // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
    // divide by 2 because we measure "round trip" time for echo
    // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
    // = 0.017*echo_time
    //
    rightPingDistanceCM = constrain(0.017*echo_time_r,5.0,50.0);
  }
}
void setServoAngle(double sDeg){
//
//  Update ServoCenter_us as Required for installation bias
//  CAREFUL: make small increments (~100) to iterate
//  100us is about 20deg (higher values --> more right steering)
//  wrong ServoCenter values can damage servo
//
    double ServoCenter_us = 1150.0;
    double ServoScale_us = 8.0;    // micro-seconds per degree
//
//  NEVER send a servo command without constraining servo motion!
//  -->large servo excursions could damage hardware or servo<--
//
    double t_us = constrain(ServoCenter_us + ServoScale_us * sDeg, ServoCenter_us-150, ServoCenter_us+150);
    steeringServo.writeMicroseconds(t_us);
}
