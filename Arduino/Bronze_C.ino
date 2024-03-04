#include <WiFiS3.h>

// WiFi Credentials
char ssid[] = "xx";
char pass[] = "xx";

// Server configuration
WiFiServer server(5200);
WiFiClient client = server.available();

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
}

void loop() {

  // Server
  client = server.available();   

  // Main Loop
  if (client.available()){
    String check = client.readString();

    //Check Communication
    //'S' equal to stop
    if (check.equals("S")){
      Serial.println("Stop");
      stop_all();

    //'G' equal to go
    } else if (check.equals("G")){
      Serial.println("Go");
      while (true) {
        //Ultrasonic
        SendSound();
        duration = pulseIn(echo, HIGH);
        cm = microsecondsToCentimeters(duration);
        //Object detected
        if (cm <= 30){
          client.print("Object Detected at ");
          client.println(String(cm) + " cm");
        }

        bool all_clearL = digitalRead(LEYE);
        bool all_clearR = digitalRead(REYE);
        
        //Track movement function
        Check(all_clearL, all_clearR, cm);

        // Add a delay to avoid overwhelming the Arduino
        delay(100);

        if (client.available()) {
          //Check new string
          String newCheck = client.readString();
          
          if (newCheck.equals("S")){
            Serial.println("Stop");
            stop_all();
             // Exit the while loop
            break; 
          }
        }
      }
    }
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

// All_clear = black, !all_clear = white
void Check(bool all_clearL, bool all_clearR, int cm) {

  // Ultrasonic
  if (cm < 15) {
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