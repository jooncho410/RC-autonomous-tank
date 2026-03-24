#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>


// RF69
#define RF_CS   10
#define RF_RST  9
#define RF_IRQ  2
#define RF_FREQ 915.0
RH_RF69 rf69(RF_CS, RF_IRQ);


// STEPPER DRIVER PINS
#define EN_PIN 8
#define L_STEP 3
#define L_DIR  4
#define R_STEP 5
#define R_DIR  6


// ULTRASONIC SENSOR
// OBST = distance threshold (cm) used in autonomous mode
#define SONAR 7
#define OBST 15.0f


// IMU SENSORS (I2C)
// LSM6: accel/gyro, LIS3MDL: magnetometer (for yaw/heading)
LSM6 imu;
LIS3MDL mag;


// MOTION STATE
unsigned long tL=0, tR=0; // next step time for L/R motor
int8_t Lcmd=0, Rcmd=0; // current commanded speed L/R
int8_t Lcmd_target=0, Rcmd_target=0; // latest command received over RF
uint8_t mode=0; // 0 = RC/manual, 1 = AUTO


// Auto-mode state variables
bool turning=false;
float targetYaw=0;
bool imuRequest=false;


// ACCELERATION LIMITING
// Ramp command toward target by ACCEL_STEP every ACCEL_DT ms
const int ACCEL_STEP = 2;
const unsigned long ACCEL_DT = 5;
unsigned long lastAccelTime = 0;


// Convert signed speed command to step interval (ms)
// map(|s| = 1~127) -> 2500~200 us
unsigned long spdInterval(int s){
 if(s==0) return 0;
 return map(abs(s), 1, 127, 2500, 200);
}


// SONAR DISTANCE MEASUREMENT (cm)
float sonar(){
 // Trigger burst
 pinMode(SONAR,OUTPUT);
 digitalWrite(SONAR,LOW); delayMicroseconds(2);
 digitalWrite(SONAR,HIGH); delayMicroseconds(10);
 digitalWrite(SONAR,LOW);


 // Wait for echo to go high
 pinMode(SONAR,INPUT);
 unsigned long start=micros();
 while(!digitalRead(SONAR)){
   if(micros()-start>30000) return 999; // no echo detected
 }


 // Measure how long echo stays high
 unsigned long t=micros();
 while(digitalRead(SONAR)){
   if(micros()-t>30000) break; // clamp very long echoes
 }


 // Distance conversion (cm)
 return (micros()-t)*0.01715;
}


unsigned long lastPing = 0;  // rate limit sonar sampling in auto mode


// Smoothly ramp current commands toward target commands
// prevents sudden speed jumps that can cause missed steps
void updateAcceleration(){
 if(millis() - lastAccelTime < ACCEL_DT) return;
 lastAccelTime = millis();


 if(Lcmd < Lcmd_target) Lcmd += ACCEL_STEP;
 if(Lcmd > Lcmd_target) Lcmd -= ACCEL_STEP;
 if(Rcmd < Rcmd_target) Rcmd += ACCEL_STEP;
 if(Rcmd > Rcmd_target) Rcmd -= ACCEL_STEP;
}


// Generate STEP pulses for each motor at the scheduled interval
void driveMotors(){
 unsigned long now=micros();


 if(Lcmd!=0 && now>=tL){
   digitalWrite(L_STEP,HIGH);
   delayMicroseconds(2); // minimum STEP high time
   digitalWrite(L_STEP,LOW);
   tL = now + spdInterval(Lcmd);
 }


 if(Rcmd!=0 && now>=tR){
   digitalWrite(R_STEP,HIGH);
   delayMicroseconds(2);
   digitalWrite(R_STEP,LOW);
   tR = now + spdInterval(Rcmd);
 }
}


// Receive latest control packet from controller:
// [L_target, R_target, mode, imuRequestFlag]
void handleRF(){
 if(rf69.available()){
   uint8_t buf[4];
   uint8_t len = sizeof(buf);


   if(rf69.recv(buf,&len)){
     Lcmd_target = (int8_t)buf[0];
     Rcmd_target = (int8_t)buf[1];
     mode = buf[2];
     imuRequest = buf[3];


     // Direction logic
     // left: <=0 one direction, right: >=0 one direction
     digitalWrite(L_DIR, Lcmd_target <= 0);
     digitalWrite(R_DIR, Rcmd_target >= 0);


     turning = false;
   }
 }
}


// Autonomous mode: simple obstacle-avoidance state machine
// If not turning: drive forward. if obstacle < OBST = begin turning
// If turning: spin in place
void autoMode(){
 if(millis() - lastPing > 150){
   lastPing = millis();
   float d = sonar();


   if(!turning){
     Lcmd_target = 100;
     Rcmd_target = 100;


     if(d < OBST){
       turning = true;
       targetYaw += 15; // intended heading target increment
     }
   }
   else{
     Lcmd_target = -100;
     Rcmd_target = 100;


     if(abs(targetYaw) < 2){
       turning = false;
     }
   }
 }
}


// If requested by controller, take a single IMU reading and transmit
void processIMURequest(){
 if(!imuRequest) return;


 imu.read();
 mag.read();


 // Roll/pitch from accelerometer; yaw from magnetometer
 float roll  = atan2(imu.a.y, imu.a.z) * 57.3;
 float pitch = atan2(-imu.a.x,
    sqrt(imu.a.y*imu.a.y + imu.a.z*imu.a.z)) * 57.3;
 float yaw   = atan2(mag.m.y, mag.m.x) * 57.3;


 uint8_t tx[6];
 int16_t r=roll*100, p=pitch*100, y=yaw*100;


 tx[0]=r>>8; tx[1]=r;
 tx[2]=p>>8; tx[3]=p;
 tx[4]=y>>8; tx[5]=y;


 rf69.send(tx,6);
 rf69.waitPacketSent();


 imuRequest = false;
}


// SETUP: initialize motor pins, I2C sensors, and RF module
void setup(){
 pinMode(EN_PIN,OUTPUT);
 pinMode(L_STEP,OUTPUT); pinMode(L_DIR,OUTPUT);
 pinMode(R_STEP,OUTPUT); pinMode(R_DIR,OUTPUT);
 digitalWrite(EN_PIN,LOW); // enable drivers (active-low)


 // Initialize IMU sensors over I2C
 Wire.begin();
 imu.init(); imu.enableDefault();
 mag.init(); mag.enableDefault();


 // Reset RF module then configure frequency/power
 pinMode(RF_RST,OUTPUT);
 digitalWrite(RF_RST,LOW); delay(10);
 digitalWrite(RF_RST,HIGH); delay(10);
 digitalWrite(RF_RST,LOW);


 rf69.init();
 rf69.setFrequency(RF_FREQ);
 rf69.setTxPower(20,true);
}


// Main loop: receive commands, optionally run auto behavior,
// then ramp + drive motors, and respond to IMU requests
void loop(){
 handleRF();


 if(mode == 1){
   autoMode();
 }


 updateAcceleration();
 driveMotors();
 processIMURequest();
}
