// Constants for PID control
const float Kp = 7;  // Proportional gain
const float Ki = 0.17; // Integral gain
const float Kd = 0.05;  
const int desiredDistance = 10;  // Desired distance from the object

// Variables for PID control
float previousError = 0;
float integral = 0;

// DC Motor Pins
// Motor A connections
const int enA = 11; 
const int in1 = 8;
const int in2 = 7;

// Motor B connections
const int enB = 10;
const int in3 = 5;
const int in4 = 6;

// Set the speed (0 = off and 255 = max speed)
int motorSpeed = 153;

// Ultrasonic Sensor Pins
const int trig = 3; // Trigger Pin
const int echo = 4; // Echo Pin
long duration, cm;



void setup() {
  Serial.begin(9600);

  // Motor control pins setup
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Set the motor speed
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);

  // Ultrasonic sensor setup
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

void loop() {
  go_forward();

  SendSound();
  duration = pulseIn(echo, HIGH);
  cm = microsecondsToCentimeters(duration);

  // Object detected
  if (cm <= 50) {
    // Calculate error for PID control
    float error = cm - desiredDistance;
   

    // Proportional term
    float proportional = Kp * error;

    float d = error - previousError;

    // Integral term
    integral = (integral + error)*Ki;

    // Calculate motor speed based on PID terms
    //int motorSpeedLeft = motorSpeed + proportional + integral;
    int motorSpeed = motorSpeed + proportional + integral + Kd*d;

    previousError = error;

    // Ensure motor speeds are within limits
    motorSpeed = constrain(motorSpeed, 0, 255);
    //motorSpeedRight = constrain(motorSpeedRight, 0, 255);

    //Serial.println(cm);
    //Serial.print(", ");
    Serial.println(motorSpeed);

    // Set motor speeds
    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed);

    if(cm <5){
      stop_all();
    }
    }
    delay(100);
}

// Ultrasonic functions
long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void SendSound() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
}

// Motors functions
void go_forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void go_backwards() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void go_right() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void go_left() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void stop_all() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
