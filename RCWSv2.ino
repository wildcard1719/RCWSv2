#include <Servo.h>
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"

Servo x_servo;
Servo y_servo;
Servo cam_servo;

int x_in = A0;
int y_in = A1;
int fire_in = A2;
int laser_in = A3;
int mode_in =10;
int fire_pin = 5;
int laser_pin = 6;
int INTERRUPT_PIN = 2;

float XXX = 90;
float YYY = 90;
float XX;
float YY;
int X;
int Y;
int fire = -1;
int laser = -1;
int mode = -1;
int angle;
int distance;
float distance_;
int scan_angle;
int distance_b;
int gimbal = 0;
int mpu_angle;
int mpu_on = 0;

int ax;
int az;
int ay;
int gx;
int gy;
int gz;
int ax_b;
int az_b;
int mpu_angle_x;
int mpu_angle_z;

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
bool blinkState = false;
bool dmpReady = false;
uint8_t devStatus; 
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

void read_angle(){
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    az = ypr[0] * 180/M_PI;
    ax = ypr[1] * 190/M_PI;
    
  }
}

void read_pin(){
  X = pulseIn(x_in, HIGH);
  Y = pulseIn(y_in, HIGH);
  fire = pulseIn(fire_in, HIGH);
  laser = pulseIn(laser_in, HIGH);
  mode = pulseIn(mode_in, HIGH);
  
  XXX = map(X, 976, 1993, 30, 150);
  
  YY = map(Y, 976, 1993, -10, 9 );
  fire = map(fire, 976, 1933, -1, 1);
  laser = map(laser, 976, 1933, -1, 1);
  mode = map(mode, 976, 1933, -1, 1);
}

void read_distance(){
  while(Serial.available()){
    if (Serial.read() == '\n'){
      distance_ = Serial.parseFloat();
      distance = distance_ * 10;
      if(distance > 120){
        distance = 130;
      }
      angle = map(distance, 0, 120, 0, 8);
    }
  }
}

void trap(){
  read_distance();
  distance_b = distance;
  while(true){
    read_pin();
    if(mode != 1){
      break;
    }
    read_distance();
    if(distance != distance_b){
      digitalWrite(fire_pin, HIGH);
    }else{
      digitalWrite(fire_pin, LOW);
    }
  }
}

void setup(){
  Serial.begin(115200);
  pinMode(x_in, INPUT);
  pinMode(y_in, INPUT);
  pinMode(fire_in, INPUT);
  pinMode(laser_in, INPUT);
  pinMode(fire_pin, OUTPUT);
  pinMode(laser_pin, OUTPUT);
  digitalWrite(fire_pin, LOW);
  x_servo.attach(9);
  y_servo.attach(4);
  cam_servo.attach(3);
  x_servo.write(90);
  y_servo.write(90);
  cam_servo.write(90);
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  pinMode(INTERRUPT_PIN, INPUT);
  mpu.initialize();
  mpu.setDLPFMode(MPU6050_DLPF_BW_5); 
  mpu.testConnection();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
  angle = 0;
  
  if(gimbal == 1){
    if(mpu_on == 0){
      read_angle();
      ax_b = ax;
      az_b = az;
      mpu_on = 1;
    }
    if(mpu_on == 1){
      read_angle();
      mpu_angle_x = (ax - ax_b);
      mpu_angle_z = (az - az_b);
    }
  }else{
    mpu_angle_z = 0;
  }
  YY = float(YY / 5.0);
  
  if ((YYY + YY) <= 145 && (YYY + YY) >= 70){
    YYY = float(YYY + float(YY));
  }
  
  if (X == 0){
    XXX = 90;
  }

  if(YYY > 90){
    angle -= map(YYY, 90, 145, 0, 10); //Y값보정
  }
  read_distance();
  Serial.println(String(distance));
  x_servo.write(XXX + mpu_angle_z * 4);
  y_servo.write(YYY + angle + mpu_angle_x + 2);
  cam_servo.write(map(YYY, 0, 180, 180, 0) - mpu_angle_x);
  
  read_pin();
  
  if(fire == 1){
    digitalWrite(fire_pin, HIGH);
  }else{
    digitalWrite(fire_pin, LOW);
  }
  if(laser == 1){
    digitalWrite(laser_pin, HIGH);
  }else{
    digitalWrite(laser_pin, LOW);
  }
  if(mode == 1){
    gimbal = 1;
  }
  if(mode == 0){
    gimbal = 0;
    mpu_on = 0;
    ax = 0;
    mpu_angle_x = 0;
  }
  if(mode == -1){
    gimbal = 0;
    mpu_on = 0;
    mpu_angle_x = 0;
  }
}
