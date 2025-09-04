#include <Servo.h>
#include <AFMotor.h>

AF_DCMotor motor1(1);
int Start = 250;
int Stop = 0;

Servo myServo;
const int servoPin = 10;
int start_pos = 90;
int cur_pos = 0;    
int new_pos = 0;
int angle = 30;

const int leftTrigPin = A1;
const int leftEchoPin = A0;
const int rightTrigPin = A3;
const int rightEchoPin = A2;
// Timing for obstacle checks
unsigned long lastObstacleCheck = 0;
const unsigned long obstacleInterval = 200; // ms
const int obstacleThreshold = 20;


// Cooldown tracking
unsigned long lastCommandTime = 0;
const unsigned long commandCooldown = 500; // 5 seconds

int baud_rate = 9600; //115200
char c = ' ';
char u = ' ';

int park_complete = 0;

void go_forward()
{
  Serial.print("\nForward - m1, m2, m3, m4: ON"); 
  motor1.run(FORWARD);
  motor1.setSpeed(Start);  
  delay(10);
}

void go_backward()
{
  Serial.print("\nBackward - m1, m2, m3, m4: ON"); 
  motor1.run(BACKWARD);
  motor1.setSpeed(Start);  
  delay(140);
}

void stop_rover()
{
  Serial.print("\nNo move - m1, m2, m3, m4: OFF"); 
  motor1.setSpeed(Stop);
  motor1.run(RELEASE);
}


int read_left_sensor()
{
  long left_duration;
  int left_sense_distance;
  digitalWrite(leftTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(leftTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(leftTrigPin, LOW);
  left_duration = pulseIn(leftEchoPin, HIGH);
  left_sense_distance = left_duration * 0.034 / 2;
  Serial.print("\nLeft Distance: ");
  Serial.println(left_sense_distance);
  return left_sense_distance;
}

int read_right_sensor()
{
  long right_duration;
  int right_sense_distance;
  digitalWrite(rightTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rightTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rightTrigPin, LOW);
  right_duration = pulseIn(rightEchoPin, HIGH);
  right_sense_distance = right_duration * 0.034 / 2;
  Serial.print("\nRight Distance: ");
  Serial.println(right_sense_distance);
  return right_sense_distance;
}

void checkObstacles() 
{
  long leftDistance = read_left_sensor(); //measureDistance(leftTrigPin, leftEchoPin);
  long rightDistance = read_right_sensor(); //measureDistance(rightTrigPin, rightEchoPin);
  
  if (leftDistance > 0 && leftDistance < obstacleThreshold) 
  {
    Serial.print("\nLeft Obstacle detected..");
    stop_rover();
    delay(1000);
    turnRight(30);
    go_forward();
    delay(1000);
    turnStraight();
  }
  
  if (rightDistance > 0 && rightDistance < obstacleThreshold) 
  {
    Serial.print("\nRight Obstacle detected..");
    stop_rover();
    delay(1000);
    turnLeft(30);
    go_forward();
    delay(1000);
    turnStraight();
  }
}

void turnRight(int degrees) 
{
  int r=0;
  Serial.print("\n\nReceived char 'R'.");
  delay(100);
  cur_pos = myServo.read();
  Serial.print("\nCurrent Servo Position: ");
  Serial.println(cur_pos);
  new_pos = cur_pos + angle;
  for (r=cur_pos; r < new_pos;)
  {
    myServo.write(r);
    r+=2;
    delay(20);
  }
  delay(15);
  cur_pos = myServo.read();
  Serial.print("\nCurrent Servo Position: ");
  Serial.println(cur_pos);
  delay(15);
}

void turnLeft(int degrees) 
{
  int l=0;
  Serial.print("\n\nReceived char 'L'.");
  delay(100);
  cur_pos = myServo.read();
  Serial.print("\nCurrent Servo Position: ");
  Serial.println(cur_pos);
  new_pos = cur_pos - angle;
  for (l=cur_pos; l >= new_pos;)
  {
    myServo.write(l);
    l-=2;
    delay(20);
  }
  delay(15);
  cur_pos = myServo.read();
  Serial.print("\nCurrent Servo Position: ");
  Serial.println(cur_pos);
  delay(15);
}

void turnStraight() 
{
  Serial.print("\n\nMake Straight.");
  delay(100);
  cur_pos = myServo.read();
  Serial.print("\nCurrent Servo Position: ");
  Serial.println(cur_pos);
  new_pos = start_pos;
  myServo.write(new_pos);
  delay(15);
  cur_pos = myServo.read();
  Serial.print("\nCurrent Servo Position: ");
  Serial.println(cur_pos);
  delay(15);
}
void parallelPark() 
{
  Serial.println("Starting parallel parking");
  stop_rover();
  turnRight(angle);
  delay(100);
  go_backward();
  delay(1000);
  stop_rover();
  turnLeft(angle);
  delay(100);
  go_forward();
  delay(1000);
  stop_rover();
  turnStraight();
  park_complete = 1;
  Serial.println("Parking complete");
}


void setup() 
{
  Serial.begin(baud_rate);
  Serial.println("\n\nRover started..");

  myServo.attach(servoPin);
  delay(10);
  myServo.write(start_pos);
  delay(100);
  cur_pos = myServo.read();
  Serial.print("\nCurrent Servo Position: ");
  Serial.println(cur_pos);
  delay(500);
  
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);

  park_complete = 0;

  Serial.println("Ready. Send commands from Raspberry Pi.");
}

void loop() 
 {
  if (park_complete == 0)
  {
    go_forward();
  }
  else
  {
    stop_rover();
  }
  
  // Check serial instantly, no blocking
  if (Serial.available() > 0) 
  {
    c = Serial.read();

    // Ignore whitespace/newline characters
    if (c == '\n' || c == '\r' || c == ' ') return;

    // Check cooldown
    if (millis() - lastCommandTime >= commandCooldown) 
    {
      lastCommandTime = millis(); // Reset cooldown
      Serial.print("Command accepted: ");
      Serial.println(c);

      if (c == 'S') 
      {
        Serial.println("\n\nStart Rover..");
        turnStraight();
        go_forward();
        park_complete = 0;
      } 
      else if (c == 'R') // Turn Right
      {
        turnRight(angle);
        go_forward();
        delay(1000);
        // turnStraight();
      } 
      
      else if (c == 'L') // Turn Left
      {
        turnLeft(angle);   
        go_forward();
        delay(1000);  
        // turnStraight(); 
      } 

      else if (c == 'P') // Parallel Parking
      {
        parallelPark();
      }

      else if (c == 'F') // Fail Safe for obstacles
      {
        stop_rover();
        go_backward();
        delay(1000);
        stop_rover();
        while (1)
        {
          u = Serial.read();
          if (u == 'L')
          {
            turnLeft(angle);
            go_forward();
            delay(1000);  
            turnStraight();
            break;
          }
          else if (u == 'R')
          {
            turnRight(angle);
            go_forward();
            delay(1000);
            turnStraight();
            break;
          }
          delay(100);
        }
      }
        
      else 
      {
        go_forward();
      }
    } 
    else 
    {
      Serial.println("Command ignored - cooldown active");
      // Flush remaining serial input
      while (Serial.available() > 0) Serial.read();
    }
  }
  // Check obstacles on a timed schedule (non-blocking)
  unsigned long currentMillis = millis();
  if (currentMillis - lastObstacleCheck >= obstacleInterval) 
  {
    Serial.print("\nChecking Obstacle..");
    delay(10);
    lastObstacleCheck = currentMillis;
    checkObstacles();
  }
}
