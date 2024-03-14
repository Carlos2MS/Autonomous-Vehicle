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
const int motorSpeed = 255;
const int motorSpeedR = 255;
double correction = 0.6;

// Ultrasonic Sensor Pins
const int trig = 3; // Trigger Pin
const int echo = 4; // Echo Pin
long duration, cm;

// Wheel Encoder
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

// Constants for control
const double targetDistance = 1.0; // Target distance in meters
const double kP = 10.0; // Proportional gain

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
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeed);

  // Ultrasonic sensor setup
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Wheel Encoder
  pinMode(WheelL, INPUT_PULLUP);
  pinMode(WheelR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WheelL), countLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WheelR), countRigth, CHANGE);
}

void loop() {
  while (true) {
    // Ultrasonic
    SendSound();
    duration = pulseIn(echo, HIGH);
    cm = microsecondsToCentimeters(duration);

    Serial.print("Distance: ");
    Serial.println(cm);

    if (ticks_L >= revolution) {
      countL++;
      distanceL = (countL * wheel_c) / 1000; // distance in m
      ticks_L = 0;
    }
    if (ticks_R >= revolution) {
      countR++;
      distanceR = (countR * wheel_c) / 1000; // distance in m
      ticks_R = 0;
    }

    // Control to maintain a constant distance
    //double error = distanceL - distanceR;
    //double correction = kP * error;

    // Adjust motor speeds based on the correction

    // Apply the control to motors


    // Object detected
    if (cm <= 20) {
      go_forward();
      if(cm <= 10){
        stop_all();
      }
      if(cm > 10 && cm < 20){
        correction = correction - 0.01;
        correction = constrain(correction, 0.3, 1);
        int leftSpeed = motorSpeed*correction;
        int rightSpeed = motorSpeedR*correction;

      setMotorSpeeds(leftSpeed, rightSpeed);
      go_forward();
      }
      
      
    } 
    else if(cm > 20 && cm <= 40){
      correction = correction + 0.01;
      correction = constrain(correction, 0, 1);

      int leftSpeed = motorSpeed*correction;
      int rightSpeed = motorSpeedR*correction;

      setMotorSpeeds(leftSpeed, rightSpeed);
      
      go_forward();
    }
    else{
      // Stop if no object detected
      stop_all();
    }

    Serial.println(correction);
    delay(100); // Add a small delay to avoid overwhelming the serial monitor
  }
}

// Wheel Encoder functions
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
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  
  analogWrite(enA, leftSpeed);
  analogWrite(enB, rightSpeed);
}

void stop_all() {
    digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void go_forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void followObject() {
  // This is where you can implement the logic to follow the object
  // Adjust motor speeds or perform any other actions
  // For simplicity, I'm just moving forward when the object is detected
  go_forward();
}
