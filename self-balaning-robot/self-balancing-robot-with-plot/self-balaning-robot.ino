#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// ================= USER OPTIONS =================

// log angle/PID to Serial Plotter
#define LOG_INPUT        1
#define MANUAL_TUNING    0
#define MOVE_BACK_FORTH  0

// minimum PWM so your yellow gear motors actually move
#define MIN_ABS_SPEED 20   

// Motor direction (flip if it accelerates the fall)
const int MOTOR_DIRECTION = -1;   // try 1 or -1

// =================================================
// ================   MPU 6050   ===================

MPU6050 mpu;

// MPU control/status vars
bool     dmpReady    = false;
uint8_t  mpuIntStatus;
uint8_t  devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t  fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// =================================================
// ===================== PID =======================

double originalSetpoint = 137.065;   // your measured balance angle
double setpoint         = originalSetpoint;
double movingAngleOffset = 0.3;      // for MOVE_BACK_FORTH mode

double input  = 0;   // current angle
double output = 0;   // PID output
int    moveState = 0;

// Auto-balance state: only drive motors when near upright
bool   balancingEnabled = false;
const double START_ANGLE_WINDOW = 6.0;   // start balancing if |error| < 6°
const double STOP_ANGLE_WINDOW  = 25.0;  // stop if |error| > 25°

#if MANUAL_TUNING
  double kp, ki, kd;
  double prevKp, prevKi, prevKd;
  PID pid(&input, &output, &setpoint, 0, 0, 0, DIRECT);
#else
  // Reasonable starting values for your tall, light chassis
  PID pid(&input, &output, &setpoint, 45.0, 0.7, 3.5, DIRECT);
#endif

// =================================================
// ============== MOTOR CONTROLLER =================

int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;

// both sides same power for now
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4,
                                 1.0,   // left factor
                                 1.0);  // right factor

// =================================================
// ==================== TIMERS =====================

long time1Hz = 0;
long time5Hz = 0;

// =================================================
// ================== INTERRUPT ====================

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// =================================================
// ===================== SETUP =====================

void setup() {
  // I2C
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection()
                 ? F("MPU6050 connection successful")
                 : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // === YOUR CALIBRATION OFFSETS ===
  mpu.setXAccelOffset(-16788);
  mpu.setYAccelOffset(605);
  mpu.setZAccelOffset(6661);
  mpu.setXGyroOffset(-175);
  mpu.setYGyroOffset(-298);
  mpu.setZGyroOffset(151);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (INT 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady  = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    // PID setup
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);            // 10 ms loop
    pid.SetOutputLimits(-180, 180);   // motor command range
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

// =================================================
// ====================== LOOP =====================

void loop() {
  if (!dmpReady) return;

  // while waiting for a fresh DMP packet, run PID using last angle
  while (!mpuInterrupt && fifoCount < packetSize) {

    if (balancingEnabled) {
      pid.Compute();
      motorController.move(MOTOR_DIRECTION * output, MIN_ABS_SPEED);
    } else {
      // keep motors stopped when not balancing
      motorController.move(0, MIN_ABS_SPEED);
      output = 0;
    }

    unsigned long currentMillis = millis();

    if (currentMillis - time1Hz >= 1000) {
      loopAt1Hz();
      time1Hz = currentMillis;
    }

    if (currentMillis - time5Hz >= 5000) {
      loopAt5Hz();
      time5Hz = currentMillis;
    }
  }

  // got an interrupt; read fresh DMP data
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount    = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {

    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // pitch in degrees, shifted to [0,360)
    input = ypr[1] * 180.0 / M_PI + 180.0;

    // --------- AUTO START/STOP BALANCING ----------
    double angleError = input - setpoint;
    double absError   = fabs(angleError);

    if (!balancingEnabled) {
      if (absError < START_ANGLE_WINDOW) {
        balancingEnabled = true;   // close enough: start balancing
      }
    } else {
      if (absError > STOP_ANGLE_WINDOW) {
        balancingEnabled = false;  // tipped too far: stop motors
      }
    }

    // ------- OPTIONAL LOGGING FOR SERIAL PLOTTER ----
    #if LOG_INPUT
      Serial.print("angle:");
      Serial.print(input);
      Serial.print('\t');

      Serial.print("setpoint:");
      Serial.print(setpoint);
      Serial.print('\t');

      Serial.print("output:");
      Serial.print(output);
      Serial.print('\t');

      Serial.print("bal:");
      Serial.println(balancingEnabled ? 1 : 0);
    #endif
  }
}

// =================================================
// =======   LOW-FREQUENCY SERVICE LOOPS   =========

void loopAt1Hz() {
  #if MANUAL_TUNING
    setPIDTuningValues();
  #endif
}

void loopAt5Hz() {
  #if MOVE_BACK_FORTH
    moveBackForth();
  #endif
}

// Move a little back/forth by shifting setpoint (disabled by default)
void moveBackForth() {
  moveState++;
  if (moveState > 2) moveState = 0;

  if (moveState == 0)
    setpoint = originalSetpoint;
  else if (moveState == 1)
    setpoint = originalSetpoint - movingAngleOffset;
  else
    setpoint = originalSetpoint + movingAngleOffset;
}

// =================================================
// ======== POT-BASED PID TUNING (OPTIONAL) ========

#if MANUAL_TUNING
void setPIDTuningValues() {
  readPIDTuningValues();
  if (kp != prevKp || ki != prevKi || kd != prevKd) {
    #if LOG_PID_CONSTANTS
      Serial.print(kp); Serial.print(", ");
      Serial.print(ki); Serial.print(", ");
      Serial.println(kd);
    #endif
    pid.SetTunings(kp, ki, kd);
    prevKp = kp; prevKi = ki; prevKd = kd;
  }
}

void readPIDTuningValues() {
  int potKp = analogRead(A0);
  int potKi = analogRead(A1);
  int potKd = analogRead(A2);

  kp = map(potKp, 0, 1023, 0, 25000)  / 100.0;  // 0–250
  ki = map(potKi, 0, 1023, 0, 100000) / 100.0;  // 0–1000
  kd = map(potKd, 0, 1023, 0, 500)    / 100.0;  // 0–5
}
#endif
