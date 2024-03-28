// DC Motor Pins
// Motor A connections
const int enA = 11; 
const int in1 = 8;
const int in2 = 7;

// Motor B connections
const int enB = 10;
const int in3 = 5;
const int in4 = 6;


// Ultrasonic Sensor Pins
const int trig = 3; // Trigger Pin
const int echo = 4; // Echo Pin
long duration, cm;

// Wheel Encoder Pins and Variables
const int WheelL = 1;
const int WheelR = 2;

const int revolution = 384;
const int wheel_d = 64; // Wheel diameter (mm)
const float wheel_c = PI * wheel_d; // Wheel circumference (mm)

double distanceL = 0; // Distance travelled by left wheel
double distanceR = 0; // Distance travelled by right wheel
double Totaldistance = 0;

unsigned long prevTime = 0;
float pwmValue = 0;
int count = 0;


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
  // Read the console input for desired velocity
  if (Serial.available() > 0) {
    float desiredVelocity = Serial.parseFloat(); // Read the float input from the console

    Serial.print("Desired Velocity: ");
    Serial.println(desiredVelocity);

    // Calculate the PWM value based on desired velocity
    if (desiredVelocity >= 0) {
      pwmValue = map(desiredVelocity, 0, 10, 0, 255); // Assuming the velocity is in the range [0, 1] m/s
      Serial.print("Mapped PWM Value: ");
      Serial.println(pwmValue);
      
      // Apply the new speed to both moto
      analogWrite(enA, pwmValue);
      analogWrite(enB, pwmValue);
    } else {
      Serial.println("Invalid velocity input. Please enter a non-negative value.");
    }
  }


  // Calculate velocity of the buggy in m/s
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - prevTime) / 1000.0; // Convert milliseconds to seconds
  prevTime = currentTime;

  // Calculate distance based on wheel encoder ticks
  float distanceTravelled = (count * wheel_c) / 1000; // Distance in m

  Totaldistance += distanceTravelled;
  Serial.print(Totaldistance);

  // Reset count for next iteration
  count = 0;
  float velocity = distanceTravelled / elapsedTime;

  // Send velocity to console for debugging
  Serial.print("Current Velocity: ");
  Serial.print(velocity);
  Serial.println(" m/s");

  // Reset distance variable
  distanceL = 0;
  distanceR = 0;
  prevTime = currentTime;

  // Your code for reading ultrasonic sensor and adjusting motion based on sensor readings
  // Here's a basic example
  SendSound(); // Send ultrasonic pulse
  delay(100); // Wait for pulse to return
  long duration = pulseIn(echo, HIGH); // Measure the time it takes for the pulse to return
  long cm = microsecondsToCentimeters(duration); // Convert the time into distance in cm

  // Adjust motion based on distance reading
  go_forward();

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

void stop_all() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
