// IR Sensor Pins
const int REYE = 13;
const int LEYE = 12;

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

//Wheel Encoder
const int WheelL = 1;
const int WheelR = 2;

const int revolution = 7;
const int wheel_d = 64; // Wheel diameter (mm)
const float wheel_c = PI * wheel_d; // Wheel circumference (mm)

int countL = 0;
double distanceL= 0;
int countR = 0;
double distanceR = 0;

unsigned long ticks_L = 0;
unsigned long ticks_R = 0;

//Speed
unsigned long previousTime = 0;
double speed = 0;


//PIB
// Constants for PID control
const float Kp = 7;  // Proportional gain
const float Ki = 0.17; // Integral gain
const float Kd = 0.05;  
const int desiredDistance = 10;  // Desired distance from the object
const int range = 50; //Object detection range in cm

// Variables for PID control
float previousError = 0;
float integral = 0;


void setup() {
  Serial.begin(9600);

  // IR sensor setup
  pinMode(LEYE, INPUT);
  pinMode(REYE, INPUT);

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

  //Wheel Encoder
  pinMode(WheelL, INPUT_PULLUP);
  pinMode(WheelR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WheelL), countLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WheelR), countRigth, CHANGE);
}

void loop() {
  unsigned long currentTime = millis();
  //Ultrasonic
  SendSound();
  duration = pulseIn(echo, HIGH);
  cm = microsecondsToCentimeters(duration);

  //Object detected- PIB
  PIBControl(cm);

  bool all_clearL = digitalRead(LEYE);
  bool all_clearR = digitalRead(REYE);

  // Calculate the time elapsed since the last distance measurement
  unsigned long elapsedTime = currentTime - previousTime;

  if(ticks_L >= revolution){
    countL++;
    distanceL = (countL*wheel_c)/1000; // distance in m
    speed = (distanceL - previousDistance) / (elapsedTime / 1000.0);
    ticks_L = 0;
  }
  if(ticks_R >= revolution){
    countR++;
    distanceR = (countR*wheel_c)/1000; // distance in m
    speed = (distanceR - previousDistance) / (elapsedTime / 1000.0);
    ticks_R = 0;
  }

  //Track movement function
  Check(all_clearL, all_clearR, cm);

  // Add a delay to avoid overwhelming the Arduino
  delay(100);
}

//Wheel Encoder functions
void countLeft() {
  ticks_L++;
}

void countRigth() {
  ticks_R++;
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

// All_clear = black, !all_clear = white
void Check(bool all_clearL, bool all_clearR, int cm) {

  // Ultrasonic
  if (cm < 5) {
    stop_all();
    return;
  }

  if (all_clearL && all_clearR) {
    go_forward();
  }
  if (all_clearL && !all_clearR) {
    go_right();
  }
  if (!all_clearL && all_clearR) {
    go_left();
  }
  if (!all_clearL && !all_clearR) {
    // Delay added for intersections
    delay(250);
    stop_all();
  }
}

void PIBControl(int cm){
  if (cm <= range){
    // Calculate error for PID control
    float error = cm - desiredDistance;
  
    // Proportional term
    float proportional = Kp * error;

    //Derivative term
    float d = error - previousError;

    // Integral term
    integral = (integral + error)*Ki;

    // Calculate motor speed based on PID terms
    motorSpeed = motorSpeed + proportional + integral + Kd*d;

    //Update Error
    previousError = error;

    //Ensure motor speeds are within limits
    motorSpeed = constrain(motorSpeed, 0, 255);

    // Set motor speeds
    analogWrite(enA, motorSpeed);
    analogWrite(enB, motorSpeed);
    }
}
