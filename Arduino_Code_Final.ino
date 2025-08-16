#include <Servo.h>

const int servoPin = 10;
const int motorPin1 = 12;
const int motorPin2 = 13;
const int enablePin = 5;

const int leftTrigPin = A2;
const int leftEchoPin = A3;
const int rightTrigPin = A0;
const int rightEchoPin = A1;

const int obstacleThreshold = 15;

Servo myServo;
int currentServoPosition = 90;

// Timing for obstacle checks
unsigned long lastObstacleCheck = 0;
const unsigned long obstacleInterval = 200; // ms

// Cooldown tracking
unsigned long lastCommandTime = 0;
const unsigned long commandCooldown = 5000; // 5 seconds

void setup() {
  Serial.begin(115200);

  myServo.attach(servoPin);
  myServo.write(currentServoPosition);
  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
  
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);

  Serial.println("Ready. Send commands from Raspberry Pi.");
}

void loop() {
  // Check serial instantly, no blocking
  if (Serial.available() > 0) {
    char c = Serial.read();

    // Ignore whitespace/newline characters
    if (c == '\n' || c == '\r' || c == ' ') return;

    // Check cooldown
    if (millis() - lastCommandTime >= commandCooldown) {
      lastCommandTime = millis(); // Reset cooldown
      Serial.print("Command accepted: ");
      Serial.println(c);

      if (c == 'B') {
        Serial.println("B received");
      } 
      else if (c == 'R') {
        Serial.println("R received");
        turnServoRelative(50);
        delay(2);
        turnServoRelative(-50);
        
      } 
      else if (c == 'L') {
        Serial.println("L received");
        turnServoRelative(-50);
        delay(2);
        turnServoRelative(50);
        
      } 
      else {
        executeCommand(c);
      }
    } 
    else {
      Serial.println("Command ignored - cooldown active");
      // Flush remaining serial input
      while (Serial.available() > 0) Serial.read();
    }
  }

  // Check obstacles on a timed schedule (non-blocking)
  unsigned long currentMillis = millis();
  if (currentMillis - lastObstacleCheck >= obstacleInterval) {
    lastObstacleCheck = currentMillis;
    checkObstacles();
  }
}

long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout
  if (duration == 0) return -1; // No reading
  return duration * 0.034 / 2;
}

void checkObstacles() {
  long leftDistance = measureDistance(leftTrigPin, leftEchoPin);
  long rightDistance = measureDistance(rightTrigPin, rightEchoPin);
  
  if (leftDistance > 0 && leftDistance < obstacleThreshold) {
    turnServoRelative(10);
  }
  
  if (rightDistance > 0 && rightDistance < obstacleThreshold) {
    turnServoRelative(10);
  }
}

void turnServoRelative(int degrees) {
  int targetPosition = currentServoPosition + degrees;
  if (targetPosition > 180) targetPosition = 180;
  if (targetPosition < 0) targetPosition = 0;

  myServo.write(targetPosition);
  delay(500); // small pause for movement

  // Move back by the same amount
  myServo.write(currentServoPosition);
  delay(500); // small pause for return

  // Keep servo position updated
  currentServoPosition = myServo.read();
}

void moveCar(int speed, bool forward, int duration_ms) {
  unsigned long startTime = millis();
  
  if (forward) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
  analogWrite(enablePin, speed);

  // Non-blocking wait
  while (millis() - startTime < (unsigned long)duration_ms) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      executeCommand(c);
    }
  }
  analogWrite(enablePin, 0);
}

void parallelPark() {
  Serial.println("Starting parallel parking");

  myServo.write(90); 
  moveCar(200, true, 500);

  myServo.write(60); 
  moveCar(200, false, 144);

  myServo.write(90); 
  moveCar(200, true, 100);

  Serial.println("Parking complete");
}

void executeCommand(char command) {
  switch (command) {
    case 'R':
      turnServoRelative(50);
      break;
    case 'G':
      turnServoRelative(-50);
      break;
    case 'C':
      turnServoRelative(30);
      break;
    case 'A':
      turnServoRelative(-30);
      break;
    case 'P':
      parallelPark();
      break;
    default:
      break;
  }
}