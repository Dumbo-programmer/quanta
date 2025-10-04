// === Pin Definitions ===
#define TRIG_LEFT 5
#define ECHO_LEFT 18
#define TRIG_RIGHT 19
#define ECHO_RIGHT 21

#define ENA 14
#define ENB 13
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33

// Variables
long duration;
int distance;
int leftDistance, rightDistance;

// === Distance Measurement Function ===
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999; // No echo = very far
  distance = duration * 0.034 / 2;
  return distance;
}

// === Motor Control ===
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 250);
  analogWrite(ENB, 250);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}

void setup() {
  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  Serial.begin(115200);
}

void loop() {
  // Get sensor readings
  leftDistance = getDistance(TRIG_LEFT, ECHO_LEFT);
  rightDistance = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Left: "); Serial.print(leftDistance);
  Serial.print(" cm | Right: "); Serial.print(rightDistance);
  Serial.println(" cm");

  // === Obstacle Avoidance Logic ===
  if (leftDistance < 20 && rightDistance < 20) {
    // Both blocked → back up and turn
    stopCar();
    delay(200);
    moveBackward();
    delay(600);
    turnRight(); // default turn
    delay(600);
  }
  else if (leftDistance < 20) {
    // Left blocked → turn right
    stopCar();
    delay(100);
    turnRight();
    delay(400);
  }
  else if (rightDistance < 20) {
    // Right blocked → turn left
    stopCar();
    delay(100);
    turnLeft();
    delay(400);
  }
  else {
    // Clear path → forward
    moveForward();
  }

  delay(50); // sensor stability
}
