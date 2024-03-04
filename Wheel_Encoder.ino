#include <WiFiS3.h>

// Time for turning (milliseconds)
int turnTime = 300;

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
const int motorSpeed = 255 * 0.6;

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

int count = 0;
double distance = 0;

unsigned long ticks_L = 0;
unsigned long ticks_R = 0;

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
  attachInterrupt(digitalPinToInterrupt(WheelR), countRight, CHANGE);
}

void loop() {
  //Ultrasonic
        SendSound();
        duration = pulseIn(echo, HIGH);
        cm = microsecondsToCentimeters(duration);

        bool all_clearL = digitalRead(LEYE);
        bool all_clearR = digitalRead(REYE);


        if(ticks_L >= revolution){
          count++;
          distance = (count*wheel_c)/10; // distance in cm
          ticks_L = 0;
        }

        Serial.println(distance);
        
        //Track movement function
        if(distance < 30){
          go_forward();
        }
        else{
          stop_all();
        }
        
        if (cm < 15) {
          stop_all();
          return;
        }

        // Add a delay to avoid overwhelming the Arduino
        delay(100);
}


void countLeft() {
  ticks_L++;
}

void countRight() {
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