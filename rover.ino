#include <Wire.h>
#include <QMC5883LCompass.h>

// =======================
// PIN DEFINITIONS
// =======================
// L293D 4 motors
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8
#define MOTOR12_PWM 11
#define MOTOR34_PWM 3

// L298N 2 motors (rear)
#define IN1 18
#define IN2 19
#define ENA 23

// Ultrasonic sensors
#define TRIG_LEFT 22
#define ECHO_LEFT 23
#define TRIG_RIGHT 24
#define ECHO_RIGHT 25

#define SOIL_SENSOR A8
#define BUZZER 9
#define LED_RED 10
#define LED_GREEN 8

// Motor bits for L293D
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

static uint8_t latch_state;

// =======================
// PARAMETERS
// =======================
int WATER_THRESHOLD = 400;
int SAFE_DISTANCE = 50;

// Speeds
int L293D_MAX_SPEED = 200;  // front/middle
int L298N_MAX_SPEED = 255;  // rear
int ACCEL_STEP = 10;        // step for acceleration

int currentSpeedL293D = 0;
int currentSpeedL298N = 0;

// =======================
// SETUP
// =======================
void setup() {
  Serial.begin(9600);

  // L293D pins
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);
  pinMode(MOTOR12_PWM, OUTPUT);
  pinMode(MOTOR34_PWM, OUTPUT);
  digitalWrite(MOTORENABLE, LOW);
  latch_state = 0;
  latch_tx();

  // L298N pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Ultrasonics
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // Indicators
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);

  Serial.println("6WD Rover Ready!");
}

// =======================
// LOOP
// =======================
void loop() {
  // Check soil
  int soil = analogRead(SOIL_SENSOR);
  Serial.print("Soil: "); Serial.println(soil);
  if (soil > WATER_THRESHOLD) {
    stopRover();
    signalWater();
    Serial.println("STOP: Water detected");
    while (1);
  }

  // Read distances
  int leftDist = getDistance(TRIG_LEFT, ECHO_LEFT);
  int rightDist = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  // Obstacle avoidance
  if (leftDist < SAFE_DISTANCE && rightDist < SAFE_DISTANCE) {
    moveBackward(L293D_MAX_SPEED, L298N_MAX_SPEED);
    delay(500);
    turnRight(250, 400);
  }
  else if (leftDist < SAFE_DISTANCE) {
    turnRight(250, 300);
  }
  else if (rightDist < SAFE_DISTANCE) {
    turnLeft(250, 300);
  }
  else {
    moveForward(L293D_MAX_SPEED, L298N_MAX_SPEED);
  }

  delay(50);
}

// =======================
// WATER SIGNAL
// =======================
void signalWater() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(BUZZER, HIGH);
    delay(300);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(BUZZER, LOW);
    delay(300);
  }
}

// =======================
// ULTRASONIC
// =======================
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 200;
  return duration * 0.034 / 2;
}

// =======================
// L293D MOTOR CONTROL
// =======================
void latch_tx() {
  digitalWrite(MOTORLATCH, LOW);
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_state);
  digitalWrite(MOTORLATCH, HIGH);
}

void motor_output(uint8_t output, uint8_t value) {
  if (value == 0) latch_state &= ~_BV(output);
  else latch_state |= _BV(output);
  latch_tx();
}

void motorForward(uint8_t m) { 
  if (m == 1) { motor_output(MOTOR1_A, 0); motor_output(MOTOR1_B, 1); }
  if (m == 2) { motor_output(MOTOR2_A, 0); motor_output(MOTOR2_B, 1); }
  if (m == 3) { motor_output(MOTOR3_A, 0); motor_output(MOTOR3_B, 1); }
  if (m == 4) { motor_output(MOTOR4_A, 0); motor_output(MOTOR4_B, 1); }
}
void motorBackward(uint8_t m) { 
  if (m == 1) { motor_output(MOTOR1_A, 1); motor_output(MOTOR1_B, 0); }
  if (m == 2) { motor_output(MOTOR2_A, 1); motor_output(MOTOR2_B, 0); }
  if (m == 3) { motor_output(MOTOR3_A, 1); motor_output(MOTOR3_B, 0); }
  if (m == 4) { motor_output(MOTOR4_A, 1); motor_output(MOTOR4_B, 0); }
}
void motorStop(uint8_t m) { 
  motor_output(MOTOR1_A, 0); motor_output(MOTOR1_B, 0);
  motor_output(MOTOR2_A, 0); motor_output(MOTOR2_B, 0);
  motor_output(MOTOR3_A, 0); motor_output(MOTOR3_B, 0);
  motor_output(MOTOR4_A, 0); motor_output(MOTOR4_B, 0);
}

// =======================
// ACCELERATED MOVEMENT
// =======================
void moveForward(int speedL293D, int speedL298N) {
  rampSpeed(speedL293D, speedL298N);
  motorForward(1); motorForward(2);
  motorForward(3); motorForward(4);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // L298N forward
  analogWrite(MOTOR12_PWM, currentSpeedL293D);
  analogWrite(MOTOR34_PWM, currentSpeedL293D);
  analogWrite(ENA, currentSpeedL298N);
}

void moveBackward(int speedL293D, int speedL298N) {
  rampSpeed(speedL293D, speedL298N);
  motorBackward(1); motorBackward(2);
  motorBackward(3); motorBackward(4);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // L298N backward
  analogWrite(MOTOR12_PWM, currentSpeedL293D);
  analogWrite(MOTOR34_PWM, currentSpeedL293D);
  analogWrite(ENA, currentSpeedL298N);
}

void stopRover() {
  motorStop(1); motorStop(2); motorStop(3); motorStop(4);
  analogWrite(IN1, LOW); analogWrite(IN2, LOW); // L298N stop
  analogWrite(MOTOR12_PWM, 0);
  analogWrite(MOTOR34_PWM, 0);
  analogWrite(ENA, 0);
  currentSpeedL293D = 0;
  currentSpeedL298N = 0;
}

// =======================
// TURNING
// =======================
void turnLeft(int speed, int duration) {
  motorBackward(1); motorBackward(2);
  motorForward(3); motorForward(4);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(MOTOR12_PWM, speed);
  analogWrite(MOTOR34_PWM, speed);
  analogWrite(ENA, speed);
  delay(duration);
  stopRover();
}

void turnRight(int speed, int duration) {
  motorForward(1); motorForward(2);
  motorBackward(3); motorBackward(4);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  analogWrite(MOTOR12_PWM, speed);
  analogWrite(MOTOR34_PWM, speed);
  analogWrite(ENA, speed);
  delay(duration);
  stopRover();
}

// =======================
// SPEED RAMP FUNCTION
// =======================
void rampSpeed(int targetL293D, int targetL298N) {
  while (currentSpeedL293D < targetL293D || currentSpeedL298N < targetL298N) {
    if (currentSpeedL293D < targetL293D) currentSpeedL293D += ACCEL_STEP;
    if (currentSpeedL298N < targetL298N) currentSpeedL298N += ACCEL_STEP;
    if (currentSpeedL293D > targetL293D) currentSpeedL293D = targetL293D;
    if (currentSpeedL298N > targetL298N) currentSpeedL298N = targetL298N;
    analogWrite(MOTOR12_PWM, currentSpeedL293D);
    analogWrite(MOTOR34_PWM, currentSpeedL293D);
    analogWrite(ENA, currentSpeedL298N);
    delay(20); // adjust smoothness
  }
}
