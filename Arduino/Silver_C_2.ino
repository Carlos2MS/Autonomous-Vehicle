#include <WiFiS3.h>

int tempSpeed = 0;
bool PIB = false;
bool GUI = false;

// WiFi Credentials
char ssid[] = "C";
char pass[] = "1234567890";

// Server configuration
WiFiServer server(5200);
WiFiClient client = server.available();

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
int motorSpeed = 255 * 0.6;

// Ultrasonic Sensor Pins
const int trig = 3; // Trigger Pin
const int echo = 4; // Echo Pin
long duration, cm;

//Wheel Encoder
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

//PIB
// Constants for PID control
const float Kp = 7;  // Proportional gain
const float Ki = 0.17; // Integral gain
const float Kd = 0.05;  
const int desiredDistance = 8;  // Desired distance from the object
const int range = 50; //Object detection range in cm

// Variables for PID control
float previousError = 0;
float integral = 0;

int UserSpeed = 0;

String speedString = "0";

void setup() {
  Serial.begin(9600);

  // Connect to WiFi
  WiFi.begin(ssid, pass);

  // Wait for WiFi connection
  int attempts = 0;
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    
    attempts++;
    if (attempts > 20) {
      Serial.println("\nFailed to connect to WiFi. Please check your credentials.");
    }
  }

  // Display IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("\nIP Address:");
  Serial.println(ip);

  // Initialize server
  server.begin();

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
  // Server
  client = server.available();   

  //Time
  unsigned long currentTime = millis();

  // Main Loop
  if (client.available()){
    String check = client.readString();

    //Check Communication
    //'S' equal to stop
    if (check.equals("S")){
      Serial.println("Stop");
      stop_all();
      PIB = false;
      GUI = false;
      motorSpeed = 255 * 0.6;
    } 
    //'G' equal to go BRONZE
    else if (check.equals("G")){
      Serial.println("Go");

      while (true) {
        //Ultrasonic
        SendSound();
        duration = pulseIn(echo, HIGH);
        cm = microsecondsToCentimeters(duration);

        bool all_clearL = digitalRead(LEYE);
        bool all_clearR = digitalRead(REYE);

        if (PIB == true){
          Serial.println("PIB");
          // Calculate error for PID control
          float error = cm - desiredDistance;
  
          // Proportional term
          float proportional = Kp * error;

          //Derivative term
          float d = error - previousError;

          // Integral term
          integral = (integral + error)*Ki;

          // Calculate motor speed based on PID terms
          int motorSpeed = motorSpeed + proportional + integral + Kd*d;

          //Update Error
          previousError = error;

          //Ensure motor speeds are within limits
          motorSpeed = constrain(motorSpeed, 0, 250);

          // Set motor speeds
          analogWrite(enA, motorSpeed);
          analogWrite(enB, motorSpeed);
        }
        
        else if (GUI == true){
          // Read Speed
          UserSpeed = speedString.toInt();
           // Convert the string to an integer
          Serial.print("Received Speed: ");
          Serial.println(UserSpeed);

          if(UserSpeed != 0){
          tempSpeed = UserSpeed;
          }

          UserSpeed = tempSpeed;

          //GUI Control
          if(UserSpeed >= 0){
          GUIControl(UserSpeed);
          }
        }

        //Track movement function
        Check(all_clearL, all_clearR, cm);

        // Calculate velocity of the buggy in m/s
        unsigned long currentTime = millis();
        float elapsedTime = (currentTime - prevTime) / 1000.0; // Convert milliseconds to seconds
        prevTime = currentTime;

        // Calculate distance based on wheel encoder ticks
        float distanceTravelled = (count * wheel_c) / 1000; // Distance in m

        Totaldistance += distanceTravelled;
        //Serial.print(Totaldistance);

        // Reset count for next iteration
        count = 0;
        float speed = distanceTravelled / elapsedTime;

        // Send velocity
        client.print("Speed ");
        client.print(String(speed));
        client.println(" m/s");  

        //Object detected
        if (cm <= 50){
          client.print("Object Detected at ");
          client.println(String(cm) + " cm");
        }

        // Reset distance variable
        distanceL = 0;
        distanceR = 0;
        prevTime = currentTime;

        // Add a delay to avoid overwhelming the Arduino
        //delay(100);

        if (client.available()) {
          //Check new string
          stop_all();
          String newCheck = client.readString();
          
          if (newCheck.equals("S") || speedString.equals("S")){
            Serial.println("Stop");
            stop_all();
            analogWrite(enA, 200);
            analogWrite(enB, 200);
            PIB = false;
            GUI = false;
             // Exit the while loop
            break; 
          }
          else if(newCheck.equals("P")){
            PIB = true;
            GUI = false;
          }
          else if(newCheck.equals("U")){
            delay(1000);
            speedString = client.readStringUntil('\n'); // Read the string until newline character
            GUI = true;
            PIB = false;
            
          }
        }
      }

    }
  } 
}

// Wheel encoder interrupt service routines
void countLeft() {
  count++;
}

void countRight() {
  count++;
}

//Distance function
void updateDistance(int ticks, int count, double distance, unsigned long ticksReset, double wheelCircumference) {
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
  if (cm < 13) {
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
    delay(200);
    stop_all();
  }
}

//GUI function
void GUIControl(int desiredVelocity){
  pwmValue = map(desiredVelocity, 0, 10, 10, 250); // Assuming the velocity is in the range [0, 1] m/s
  //Serial.print("Mapped PWM Value: ");
  //Serial.println(pwmValue);
      
  // Apply the new speed to both moto
  analogWrite(enA, pwmValue);
  analogWrite(enB, pwmValue);
}