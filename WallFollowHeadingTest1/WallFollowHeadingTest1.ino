// WallFollowHeadingTest1.ino
// BDK:ESE421:2019C
// Motor PWM control
// Using SN754410 Dual H-Bridge
// Servo steering
// 3 Ping Sensors
// IMU
//

#include <Servo.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

//
// all of the pins are defined here
//
#define SERVOPIN 7
#define SERVOPOT A7
#define SERVOSCALEUS 10
#define SERVOMAXUS 400
#define FORWARDPIN 8 // HIGH for FWD; LOW for REV
#define REVERSEPIN 9 // LOW for FWD; HIGH for REV
#define LEFTMOTORPIN 10 // Left Motor Speed Control
#define RIGHTMOTORPIN 11 // Right Motor Speed Control
#define MOTORPOT A8
#define SETUP 0  // do initial setup stuff
#define ADJUST 1 // adjust the settings
#define NOOP 2 // do nothing; return latest status
#define NORMAL 3 // do usual task
#define LEFT 0
#define CENTER 1
#define RIGHT 2
#define PINGDMAX 100
#define PINGDADJUST 50
#define PINGDMIN 2
#define PINGTRIGPINS {23, 22, 6} // ping sensor trigger pin (output from Arduino)
#define PINGECHOPINS {25, 24, 5} // ping sensor echo pin (input to Arduino)
#define PINGEGRNDPINS {27, 26, 4} // ping sensor ground pin (use digital pin as ground)
#define WALLDISTCMD 30.0
#define WALLDISTLOST 60.0
#define STEERMAXDEG 25.0
#define KPSI 2.0
#define KY 0.5
#define CMDPSIMAX 10.0
#define OMGPSI 0.25
//
// IMU uses SPI -- here are the pins on the Mega
// (Pins 49 & 47 are user selection)
//
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega
#define AX 0
#define AY 1
#define AZ 2
#define OMEGAX 3
#define OMEGAY 4
#define OMEGAZ 5
#define AZBIAS 6

void setup() {
    Serial.begin(115200);       // for sending info to the terminal
    Serial.println("WallFollowHeadingTest1");

    getPingDistance(LEFT,SETUP);
    getPingDistance(CENTER,SETUP);
    getPingDistance(RIGHT,SETUP);

    setServoAngle(0.0,SETUP);

    setMotorCommand(0.0,SETUP);

    getIMU(OMEGAZ,SETUP);

    Serial.println("Setup Complete.");
}
//////////////////////////////////////////////////////////////////
void loop() {
//////////////////////////////////////////////////////////////////
//  put all static variables at the top
//
    static double hatPsiDEG = 0.0;
    static double microsLast = micros();

    double dTus = micros() - microsLast;
    microsLast = micros();
//////////////////////////////////////////////////////////////////
//  go into adjustment mode whenever we encounter a forward obstacle
//
    if (getPingDistance(CENTER,NORMAL) < PINGDADJUST) {
        Serial.println("ADJUST");
        setMotorCommand(0.0,ADJUST);
        setServoAngle(0.0,ADJUST);
        getIMU(OMEGAZ,ADJUST);
        Serial.println();
        delay(50);
        microsLast = micros();
        hatPsiDEG = 0.0; // reset heading when holding
        return;
    }
//////////////////////////////////////////////////////////////////
//  drive at nominal speed
//
    setMotorCommand(50.0,NORMAL);
//////////////////////////////////////////////////////////////////
//  update heading estimate with integral of yaw rate
//
    double rateYawDPS = getIMU(OMEGAZ,NORMAL);
    hatPsiDEG += -rateYawDPS * 0.000001 * dTus;
//////////////////////////////////////////////////////////////////
//  steer to fixed distance from right wall when visible
//  otherwise hold heading at zero
//
    double rightWallDistCM = getPingDistance(RIGHT,NORMAL);
    double cmdPsiDEG = 0.0;
    if (rightWallDistCM < WALLDISTLOST) { 
        cmdPsiDEG = constrain(KY*(rightWallDistCM - WALLDISTCMD),-CMDPSIMAX,CMDPSIMAX);
        //
        // assume feedback is working and push estimated heading toward zero
        //
        hatPsiDEG += OMGPSI*(0.0 - hatPsiDEG) * 0.000001 * dTus;
    }
    double steeringDeg = constrain(KPSI*(cmdPsiDEG - hatPsiDEG),-STEERMAXDEG,STEERMAXDEG);
    setServoAngle(steeringDeg,NORMAL);
    Serial.println(steeringDeg);
}

////////////////////////////////////////////////////////////
// IMU Sensor -- read accel [m/s^2] and angular velocity [dps]
////////////////////////////////////////////////////////////
double getIMU(byte signalFlag, byte actionFlag) {
    static Adafruit_LSM9DS1 myIMU;
    static double dataIMU[] = {0, 0, 0,    0, 0, 0, 10.0};
    static double biasIMU[] = {0, 0, 10.0, 0, 0, 0, 0};
    if (actionFlag == NOOP) {           // do nothing--return requested value
        return dataIMU[signalFlag]-biasIMU[signalFlag];
    }
    else if (actionFlag == SETUP) {           // setup = connect pins & set ranges
        myIMU = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);
        delay(1000);
        if (!myIMU.begin()) {
            Serial.println("IMU Setup Failure...");
            while (1);
        }
        else {
            Serial.println("IMU Setup Complete");
        }
        myIMU.setupAccel(myIMU.LSM9DS1_ACCELRANGE_2G);
        myIMU.setupMag(myIMU.LSM9DS1_MAGGAIN_4GAUSS);
        myIMU.setupGyro(myIMU.LSM9DS1_GYROSCALE_245DPS);
    }
    else if (actionFlag == ADJUST) {  // update biases
        myIMU.read();  /* ask it to read in the data */
        sensors_event_t a, m, g, temp;
        myIMU.getEvent(&a, &m, &g, &temp);
        const double bandwidthLP = 0.05;
        Serial.print("  IMU BIAS: "); 
        biasIMU[AX] += bandwidthLP*(a.acceleration.x-biasIMU[AX]); Serial.print(biasIMU[AX]); 
        biasIMU[AY] += bandwidthLP*(a.acceleration.y-biasIMU[AY]); Serial.print(" "); Serial.print(biasIMU[AY]);
        biasIMU[AZ] += bandwidthLP*(a.acceleration.z-biasIMU[AZ]); Serial.print(" "); Serial.print(biasIMU[AZ]);
        biasIMU[OMEGAX] += bandwidthLP*(g.gyro.x-biasIMU[OMEGAX]); Serial.print(" "); Serial.print(biasIMU[OMEGAX]);
        biasIMU[OMEGAY] += bandwidthLP*(g.gyro.y-biasIMU[OMEGAY]); Serial.print(" "); Serial.print(biasIMU[OMEGAY]);
        biasIMU[OMEGAZ] += bandwidthLP*(g.gyro.z-biasIMU[OMEGAZ]); Serial.print(" "); Serial.print(biasIMU[OMEGAZ]);
        biasIMU[AZBIAS] = 0.0;
    }
    else if (actionFlag == NORMAL) {       // normal = new reading & return chosen signal
    }
    myIMU.read();  /* ask it to read in the data */
    sensors_event_t a, m, g, temp;
    myIMU.getEvent(&a, &m, &g, &temp);
    dataIMU[AX] = a.acceleration.x;
    dataIMU[AY] = a.acceleration.y;
    dataIMU[AZ] = a.acceleration.z;
    dataIMU[OMEGAX] = g.gyro.x;
    dataIMU[OMEGAY] = g.gyro.y;
    dataIMU[OMEGAZ] = g.gyro.z;
    dataIMU[AZBIAS] = biasIMU[AZ];

    return dataIMU[signalFlag];
}

////////////////////////////////////////////////////////////
// Motor Command -- set Motor Speed (in percent)
////////////////////////////////////////////////////////////
byte setMotorCommand(double speedPct, byte actionFlag){
    static byte motorPWM = 0;
    static byte biasPWM = 0;
    pinMode(LEFTMOTORPIN,OUTPUT); analogWrite(LEFTMOTORPIN,motorPWM);
    pinMode(RIGHTMOTORPIN,OUTPUT); analogWrite(RIGHTMOTORPIN,motorPWM);

    if (actionFlag == NOOP) {           // do nothing--return most recent value
        return motorPWM;
    }
    else if (actionFlag == SETUP) { // setup = set pin modes & initial values
        Serial.println("Motor Setup");
        pinMode(FORWARDPIN,OUTPUT); digitalWrite(FORWARDPIN,1);
        pinMode(REVERSEPIN,OUTPUT); digitalWrite(REVERSEPIN,0);
        pinMode(LEFTMOTORPIN,OUTPUT);
        pinMode(RIGHTMOTORPIN,OUTPUT);
        delay(1000);
    }
    else if (actionFlag == ADJUST) { // adjust = read bias from pot;
        speedPct = 0.0;
        biasPWM = constrain(0.25*analogRead(MOTORPOT),0,100);
        Serial.print("    Motor Bias PWM = "); Serial.print(biasPWM);
    }
    else if (actionFlag == NORMAL) { // nothing special needed for normal operation
    }
    //
    // determine direction and PWM command
    //
    byte directionFlag = (speedPct >= 0);
    digitalWrite(FORWARDPIN,directionFlag);
    digitalWrite(REVERSEPIN,!directionFlag);
    speedPct = abs(speedPct);
    motorPWM = constrain(1.0*biasPWM+0.01*(255-biasPWM)*speedPct,0,255);
    analogWrite(LEFTMOTORPIN,motorPWM);
    analogWrite(RIGHTMOTORPIN,motorPWM);

    return motorPWM;
}

////////////////////////////////////////////////////////////
// Servo Actuator -- set wheel steering angle
////////////////////////////////////////////////////////////
double setServoAngle(double sDeg, byte actionFlag){
    static double t_us = 0.0;
    static double servoCenter_us = 1350.0;  // default value can be changed in SETUP
    static Servo steeringServo;

    if (actionFlag == NOOP) {           // do nothing--return most recent value
        return t_us;
    }
    else if (actionFlag == SETUP) {           // setup = attach & center the servo
        Serial.println("Servo Setup");
        steeringServo.attach(SERVOPIN);
        delay(1000);
    }
    else if (actionFlag == ADJUST) {
        servoCenter_us = 800.0 + analogRead(SERVOPOT);        
        Serial.print("    Servo Center us = "); Serial.print(servoCenter_us);
    }
    else if (actionFlag == NORMAL) {       // nothing special needed for normal operation
    }
    t_us = constrain(servoCenter_us + SERVOSCALEUS * sDeg, servoCenter_us-SERVOMAXUS, servoCenter_us+SERVOMAXUS);
    steeringServo.writeMicroseconds(t_us);

    return t_us;
}

////////////////////////////////////////////////////////////
// Ping Sensor -- perform action on selected distance sensor
////////////////////////////////////////////////////////////
double getPingDistance(byte iPing, byte actionFlag) {
    static double pingDistanceCM[] = {0.0, 0.0, 0.0};

    byte trigPins[] = PINGTRIGPINS;
    byte echoPins[] = PINGECHOPINS;
    byte grndPins[] = PINGEGRNDPINS;

    if (actionFlag == NOOP) {           // do nothing--return most recent value
        return pingDistanceCM[iPing];
    }
    else if (actionFlag == SETUP) {           // setup = initial pins then read sensor
        Serial.println("Ping Setup");
        delay(1000);
        pinMode(grndPins[iPing],OUTPUT); digitalWrite(grndPins[iPing],LOW);      
        pinMode(trigPins[iPing],OUTPUT);
        pinMode(echoPins[iPing],INPUT);
    }
    else if (actionFlag == NORMAL) {       // nothing special needed for normal operation
    }
    //
    digitalWrite(trigPins[iPing], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[iPing], HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPins[iPing], LOW);
    //
    // The echo pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    // 10000 us timeout implies maximum distance is ~85cm
    //
    long timeout_us = 120*PINGDMAX;
    unsigned long echo_time;
    echo_time = pulseIn(echoPins[iPing], HIGH, timeout_us);
    if (echo_time == 0) {
        return 2*PINGDMAX;
    }
    else {
      pingDistanceCM[iPing] = constrain(0.017*echo_time,PINGDMIN,PINGDMAX);
    }
    //
    // return the distance in centimeters
    // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
    // divide by 2 because we measure "round trip" time for echo
    // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
    // = 0.017*echo_time
    //
    return pingDistanceCM[iPing];
}
