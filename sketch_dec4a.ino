

#include <SimpleFOC.h>  //must downgrade to 2.3.3
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Bluepad32.h>
#include <ESP32Servo.h>

Servo drive;

ControllerPtr myController;
void onConnectedController(ControllerPtr ctl) { myController = ctl; }
void onDisconnectedController(ControllerPtr ctl) { myController = nullptr; }

BLDCMotor motorT = BLDCMotor(7);  //turn
BLDCMotor motorB = BLDCMotor(7);  //balance
BLDCDriver3PWM driverB = BLDCDriver3PWM(27, 26, 25, 33);
BLDCDriver3PWM driverT = BLDCDriver3PWM(19, 18, 17, 16);
Encoder encT = Encoder(4, 5, 512);
Encoder encB = Encoder(13, 14, 512);
void doAT() { encT.handleA(); }
void doBT() { encT.handleB(); }
void doAB() { encB.handleA(); }
void doBB() { encB.handleB(); }

MPU6050 mpu;

uint8_t devStatus;
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer

Quaternion q;         // [w, x, y, z]         Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 gy;       // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float ypr[3];

int lastT;

// Complementary filter and angle limits
//float ANGLE_FILTER_G = 0.999;  // gyro portion [0...1]
uint8_t ANGLE_LIMIT = 30;      // stop program / start rise-up [deg]
//float RISEUP_END_ANGLE = 5.0;  // stop rise-up and start balancing [deg]

// PID constants
float KP = 1;
float KI = 0;
float KD = 0;

// --- Angle fix rate ---
float ANGLE_FIXRATE = 0.02;
float ANGLE_FIXRATE_2 = 0.0;

bool pidEN= false;

bool xDown = false;
void dumpGamepad(ControllerPtr ctl) {
  int range = 200;
  if (abs(ctl->axisRY()) > 30) {
    drive.writeMicroseconds(map(ctl->axisRY(), -511, 512, 1500 - range, 1500 + range));
  }
  if(ctl->x() && !xDown){
    xDown = true;
  } else if(xDown && !ctl->x()){
    xDown = false;
    pidEN=!pidEN;
    Serial.println("pid en");
  }
}
void processControllers() {
  if (myController && myController->isConnected() && myController->hasData()) {
    if (myController->isGamepad()) {
      dumpGamepad(myController);
    }
  }
}

float prevAngle = 0;

void setup() {

  Serial.begin(115200);
  Wire.begin(22,21);
  //Wire.setClock(400000);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  drive.setPeriodHertz(50);  // standard 50 hz servo
  drive.attach(23, 1000, 2000);
  //MOTOR IS VERY FAST DO NOT GO ALL THE WAY TO LIMITS

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  BP32.setup(&onConnectedController, &onDisconnectedController);

  BP32.forgetBluetoothKeys();
  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();


  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(960);
  mpu.setYGyroOffset(498);
  mpu.setZGyroOffset(2364);
  mpu.setXAccelOffset(129);
  mpu.setYAccelOffset(47);
  mpu.setZAccelOffset(-15);


  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));  //Turning ON DMP
    mpu.setDMPEnabled(true);


    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready!"));
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  encT.quadrature = Quadrature::ON;
  encT.init();
  encT.enableInterrupts(doAT, doBT);
  motorT.linkSensor(&encT);
  driverT.voltage_power_supply = 12;
  driverT.init();
  motorT.linkDriver(&driverT);
  motorT.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorT.controller = MotionControlType::torque;
  motorT.LPF_velocity.Tf = 0.05f;
  motorT.voltage_limit = 5;  //keep low for now
  motorT.init();
  motorT.initFOC();

  encB.quadrature = Quadrature::ON;
  encB.init();
  encB.enableInterrupts(doAB, doBB);
  motorB.linkSensor(&encB);
  driverB.voltage_power_supply = 12;
  driverB.init();
  motorB.linkDriver(&driverB);
  motorB.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorB.controller = MotionControlType::torque;
  motorB.LPF_velocity.Tf = 0.05f;
  motorB.voltage_limit = 8;  //keep low for now
  motorB.init();
  motorB.initFOC();

  delay(1000);  //wait a second for everyhting to start
  lastT = millis();
  
}

float pid(int dt, float prevAngle) {

  float integral;
  float error;
  float derivative;
  float prevError;
  float PIDoutput;
  float current = ypr[1];
  float SetPoint = 0;  //vertical

  if (SetPoint < current) {
    SetPoint += ANGLE_FIXRATE * dt;
  } else {
    SetPoint -= ANGLE_FIXRATE * dt;
  }

  SetPoint = ANGLE_FIXRATE * encB.getVelocity() * dt;
  prevAngle = current;
  // --- PID controller ---
  error = SetPoint - current;
  integral += error * dt;
  derivative = (error - prevError) / dt;
  prevError = error;

  PIDoutput = 0.1*(KP * error + KI * integral + KD * derivative);

  // Limit range max voltage is 10
  Serial.print("Angle: ");
  Serial.print(encB.getVelocity());
  Serial.print(" PID output: ");
  Serial.println(PIDoutput);
  return constrain(PIDoutput, -8, 8);
}

void readKPID(String in) {
  //split string "1,2,3,4,5" int kp = 1, ki = 2, kd = 3, anglefixrates 1&2

  int p1,p2, p3, p4;
  p1 = in.indexOf(',');
  p2 = in.indexOf(',',p1+1);
  p3 = in.indexOf(',',p2+1);
  p4 = in.indexOf(',',p3+1);
  KP = in.substring(0, p1).toFloat();
  KI = in.substring(p1+1, p2).toFloat();
  KD = in.substring(p2+1, p3).toFloat();   
  ANGLE_FIXRATE = in.substring(p3+1, p4).toFloat();
  ANGLE_FIXRATE_2 = in.substring(p4+1, in.length()).toFloat();
  Serial.println("kp: " + (String) KP + "  ki: " + (String) KI + "  kd: " + (String) KD + " af1: " + (String) ANGLE_FIXRATE + " af2: " + (String) ANGLE_FIXRATE_2);


}

void loop() {
  motorT.loopFOC();
  motorB.loopFOC();

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ypr[1] = ypr[1] *180/M_PI;
  }

  if(Serial.available()) readKPID(Serial.readString());

  if (BP32.update()) processControllers();

  if (millis() - lastT > 50) {
    lastT = millis();
    if (abs(ypr[1]) < ANGLE_LIMIT && pidEN) {
      motorB.move(pid(50, ypr[1]));
    } else {  //give up and let it fall
      motorB.move(0);
    }
  }
}






