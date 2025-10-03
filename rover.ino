#include <Wire.h>
#include <QMC5883LCompass.h>

// =======================
// PIN DEFINITIONS
// =======================

// L293D (front 4 motors)
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8
#define MOTOR12_PWM 11
#define MOTOR34_PWM 3

// L298N (rear 2 motors)
#define L298_IN1 18
#define L298_IN2 19
#define L298_ENA 23
#define L298_IN3 20
#define L298_IN4 21
#define L298_ENB 25

// Ultrasonic sensors
#define TRIG_LEFT 22
#define ECHO_LEFT 23
#define TRIG_RIGHT 24
#define ECHO_RIGHT 25

// Soil sensor
#define SOIL_SENSOR A8

// Buzzer and LEDs
#define BUZZER 9
#define LED_RED 10
#define LED_GREEN 8

// Motor bits (L293D)
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

static uint8_t latch_state;

// PARAMETERS
int WATER_THRESHOLD = 400;
int SAFE_DISTANCE = 50;

// ACCELERATION SETTINGS
int currentSpeed12 = 0;
int currentSpeed34 = 0;
int currentSpeedL298A = 0;
int currentSpeedL298B = 0;
int ACC_STEP = 5; // Speed increment per loop

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
  pinMode(L298_IN1, OUTPUT);
  pinMode(L298_IN2, OUTPUT);
  pinMode(L298_ENA, OUTPUT);
  pinMode(L298_IN3, OUTPUT);
  pinMode(L298_IN4, OUTPUT);
  pinMode(L298_ENB, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // Other sensors
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);

  Serial.println("Simple 6WD Rover Ready!");
}

// =======================
// LOOP
// =======================
void loop() {
  // Soil sensor
  int soil = analogRead(SOIL_SENSOR);
  Serial.print("Soil: "); Serial.println(soil);
  if (soil > WATER_THRESHOLD) {
    stopRover();
    signalWater();
    Serial.println("Stop due to water");
    while (1); // stop forever
  }

  // Ultrasonic distances
  int leftDist = getDistance(TRIG_LEFT, ECHO_LEFT);
  int rightDist = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  // Obstacle avoidance
  if (leftDist < SAFE_DISTANCE && rightDist < SAFE_DISTANCE) {
    moveBackward(200);
    delay(500);
    turnRight(180, 400);
  } else if (leftDist < SAFE_DISTANCE) {
    turnRight(180, 300);
  } else if (rightDist < SAFE_DISTANCE) {
    turnLeft(180, 300);
  } else {
    moveForward(255);
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

void motorBackward(uint8_t m) { 
  if (m == 1) { motor_output(MOTOR1_A, 1); motor_output(MOTOR1_B, 0); }
  if (m == 2) { motor_output(MOTOR2_A, 1); motor_output(MOTOR2_B, 0); }
  if (m == 3) { motor_output(MOTOR3_A, 1); motor_output(MOTOR3_B, 0); }
  if (m == 4) { motor_output(MOTOR4_A, 1); motor_output(MOTOR4_B, 0); }
}
void motorForward(uint8_t m) { 
  if (m == 1) { motor_output(MOTOR1_A, 0); motor_output(MOTOR1_B, 1); }
  if (m == 2) { motor_output(MOTOR2_A, 0); motor_output(MOTOR2_B, 1); }
  if (m == 3) { motor_output(MOTOR3_A, 0); motor_output(MOTOR3_B, 1); }
  if (m == 4) { motor_output(MOTOR4_A, 0); motor_output(MOTOR4_B, 1); }
}
void motorStop(uint8_t m) { 
  motor_output(MOTOR1_A, 0); motor_output(MOTOR1_B, 0);
  motor_output(MOTOR2_A, 0); motor_output(MOTOR2_B, 0);
  motor_output(MOTOR3_A, 0); motor_output(MOTOR3_B, 0);
  motor_output(MOTOR4_A, 0); motor_output(MOTOR4_B, 0);
}

// =======================
// L298N MOTOR CONTROL
// =======================
void rearForward(int speed) {
  digitalWrite(L298_IN1, HIGH); digitalWrite(L298_IN2, LOW);
  digitalWrite(L298_IN3, HIGH); digitalWrite(L298_IN4, LOW);
  acceleratePWM(L298_ENA, currentSpeedL298A, speed);
  acceleratePWM(L298_ENB, currentSpeedL298B, speed);
}

void rearBackward(int speed) {
  digitalWrite(L298_IN1, LOW); digitalWrite(L298_IN2, HIGH);
  digitalWrite(L298_IN3, LOW); digitalWrite(L298_IN4, HIGH);
  acceleratePWM(L298_ENA, currentSpeedL298A, speed);
  acceleratePWM(L298_ENB, currentSpeedL298B, speed);
}

void rearStop() {
  analogWrite(L298_ENA, 0);
  analogWrite(L298_ENB, 0);
  currentSpeedL298A = 0;
  currentSpeedL298B = 0;
}

// =======================
// SMOOTH ACCELERATION
// =======================
void acceleratePWM(int pin, int &current, int target) {
  if (current < target) current += ACC_STEP;
  if (current > target) current -= ACC_STEP;
  if (current > 255) current = 255;
  if (current < 0) current = 0;
  analogWrite(pin, current);
}

// =======================
// ROVER MOVEMENT
// =======================
void moveForward(int speed) {
  motorForward(1); motorForward(2);
  motorForward(3); motorForward(4);
  rearForward(speed);
  acceleratePWM(MOTOR12_PWM, currentSpeed12, speed);
  acceleratePWM(MOTOR34_PWM, currentSpeed34, speed);
}

void moveBackward(int speed) {
  motorBackward(1); motorBackward(2);
  motorBackward(3); motorBackward(4);
  rearBackward(speed);
  acceleratePWM(MOTOR12_PWM, currentSpeed12, speed);
  acceleratePWM(MOTOR34_PWM, currentSpeed34, speed);
}

void stopRover() {
  motorStop(1); motorStop(2); motorStop(3); motorStop(4);
  rearStop();
  acceleratePWM(MOTOR12_PWM, currentSpeed12, 0);
  acceleratePWM(MOTOR34_PWM, currentSpeed34, 0);
}

void turnLeft(int speed, int duration) {
  motorBackward(1); motorBackward(2);
  motorForward(3); motorForward(4);
  rearBackward(speed);
  acceleratePWM(MOTOR12_PWM, currentSpeed12, speed);
  acceleratePWM(MOTOR34_PWM, currentSpeed34, speed);
  delay(duration);
  stopRover();
}

void turnRight(int speed, int duration) {
  motorForward(1); motorForward(2);
  motorBackward(3); motorBackward(4);
  rearForward(speed);
  acceleratePWM(MOTOR12_PWM, currentSpeed12, speed);
  acceleratePWM(MOTOR34_PWM, currentSpeed34, speed);
  delay(duration);
  stopRover();
}
