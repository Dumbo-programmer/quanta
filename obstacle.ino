// === Motor Pins (L298N) ===
#define IN1 18
#define IN2 19
#define IN3 21
#define IN4 22
#define ENA 23   // Motor A PWM
#define ENB 25   // Motor B PWM

// === Ultrasonic Sensors ===
#define TRIG_LEFT 26
#define ECHO_LEFT 27
#define TRIG_RIGHT 14
#define ECHO_RIGHT 15

// === Motor Speed (0-255) ===
int motorSpeed = 255; // Full throttle

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ultrasonic sensors
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
}

// === Ultrasonic Function ===
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1; // No echo
  return duration * 0.034 / 2;  // cm
}

// === Motor Control ===
void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnLeft(int speedA, int speedB) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void turnRight(int speedA, int speedB) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// === Main Loop ===
void loop() {
  float distLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  float distRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Dist L: "); Serial.print(distLeft);
  Serial.print(" cm | Dist R: "); Serial.print(distRight);
  Serial.println(" cm");

  // === Obstacle Avoidance Logic ===
  if ((distLeft > 0 && distLeft < 15) && (distRight > 0 && distRight < 15)) {
    // Obstacle straight ahead: try to back up slightly
    stopMotors();
    delay(200);
    turnRight(motorSpeed, motorSpeed / 2); // Turn a bit right
    delay(400);
  }
  else if (distLeft > 0 && distLeft < 15) {
    // Obstacle on left, turn right
    turnRight(motorSpeed, motorSpeed / 2);
    delay(300);
  }
  else if (distRight > 0 && distRight < 15) {
    // Obstacle on right, turn left
    turnLeft(motorSpeed / 2, motorSpeed);
    delay(300);
  }
  else {
    // No obstacle, go forward
    forward();
  }

  delay(50); // small delay for stability
}
