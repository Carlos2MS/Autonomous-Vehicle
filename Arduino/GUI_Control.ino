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
double previousDistanceR = 0;
double previousDistanceL = 0;
double speedR = 0;
double speedL = 0;
double speed = 0;

//GUI
double Kg = 2;


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

  //Wheel Encoder
  pinMode(WheelL, INPUT_PULLUP);
  pinMode(WheelR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WheelL), countLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WheelR), countRight, CHANGE);
}

void loop() {
  go_foward();
  
  unsigned long currentTime = millis();

  // Calculate the time elapsed since the last distance measurement
  unsigned long elapsedTime = currentTime - previousTime;

  //Update distance for left wheel
  updateDistance(ticks_L, countL, distanceL, ticks_L, wheel_c);

  //Update distance for right wheel
  updateDistance(ticks_R, countR, distanceR, ticks_R, wheel_c);

  if(distanceL > distanceR){
    //client.print("Distance ");
    //client.println(distanceL);
    speedL = (distanceL - previousDistanceL) / (elapsedTime / 1000.0);
  }
  else{
    //client.print("Distance ");
    //client.println(distanceR);
    speedR = (distanceR - previousDistanceR) / (elapsedTime / 1000.0);
  }

  if(speedL > speedR){
    speed = speedL;
  }
  else{
    speed = speedR;
  }

  //Check if there's a user input
  if (Serial.available() > 0) {
    // Read the incoming byte
    int UserIn = Serial.parseInt();
    
    if (UserIn >0) {
      previousUserSpeed = UserSpeed;
      UserSpeed = UserIn;  // Update the user speed
    
      // Inform the user about the change in speed
      Serial.print("User speed updated to ");
      Serial.println(UserSpeed);
    } else {
      Serial.println("Invalid speed input.");
    }
    
    // Wait for a brief moment to prevent multiple readings
    delay(100);
  }

  //Use GUI
  GUIControl(speed, UserSpeed);

  //Update variables
  previousDistanceL = distanceL;
  previousDistanceR = distanceR;
  previousTime = currentTime;

  //Add a delay to avoid overwhelming the Arduino
  delay(100);
}

//Wheel Encoder functions
void countLeft() {
  ticks_L++;
}

void countRight() {
  ticks_R++;
}

//Distance function
void updateDistance(int &ticks, int &count, double &distance, unsigned long &ticksReset, double wheelCircumference) {
  if (ticks >= revolution) {
    count++;
    distance = (count * wheelCircumference) / 1000.0; // Convert to meters
    ticks = 0;
  }
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

//GUI function
void GUIControl(double speed,double Userspeed){
  if (speed > UserSpeed){
    motorSpeed = motorSpeed - Kg; 
    motorSpeed = constrain(motorSpeed, 0, 255);
  }
  else if (speed < UserSpeed){
    motorSpeed = motorSpeed + Kg; 
    motorSpeed = constrain(motorSpeed, 0, 255);
  }
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}
