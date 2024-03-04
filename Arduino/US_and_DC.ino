// IR PINS
const int REYE = 13;
const int LEYE = 12;

// DC MOTORS
// Motor A connections
const int enA = 11; 
const int in1 = 8;
const int in2 = 7;
 
// Motor B connections
const int enB = 10;
const int in3 = 5;
const int in4 = 6;

// Set the speed (0 = off and 255 = max speed)
const int motorSpeed = 255*0.6;


// ULTRASONIC PINS
const int trig = 3; // Trigger Pin
const int echo = 4; // Echo Pin
long duration, cm;


void setup() {
  Serial.begin(9600);
  pinMode( LEYE, INPUT );
  pinMode( REYE, INPUT );

  // Motor control pins are outputs
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

  //ULTRASONIC
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

}

void loop() {
  
  
  bool all_clearL = digitalRead( LEYE );
  bool all_clearR = digitalRead( REYE );

  SendSound();
  duration = pulseIn(echo, HIGH);
  cm = microsecondsToCentimeters(duration);

  //Debug Print cm
  Serial.println(cm);

  Check(all_clearL,all_clearR,cm);
  delay(100);
  
}

//Ultrasonic functions

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

void SendSound(){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
}


// Motors functions
void go_forward() {
  //Serial.println("Forward");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void go_backwards() {
  //Serial.println("Backward");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void go_right() {
  //Serial.println("Right");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void go_left() {
  //Serial.println("Left");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void stop_all() {
  //Serial.println("Stop All");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// all_clear = black
// !all_clear = white

void Check(bool all_clearL,bool all_clearR, int cm){
  if(cm < 12){
    stop_all();
    return;
  }
  if( all_clearL && all_clearR ){
  go_forward();
  }
  if(all_clearL && !all_clearR){
    go_right();
  }
  if(!all_clearL && all_clearR){
    go_left();
  }
  if(!all_clearL && !all_clearR ){
//dealy added for intersections
  delay(250);
  stop_all();
  }
}