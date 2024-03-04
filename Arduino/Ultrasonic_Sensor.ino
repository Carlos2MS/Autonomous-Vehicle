const int trig = 3; // Trigger Pin of Ultrasonic Sensor
const int echo = 4; // Echo Pin of Ultrasonic Sensor

long duration, cm;

void setup() {
   Serial.begin(9600);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
}

void loop() {
   
   digitalWrite(trig, LOW);
   delayMicroseconds(2);
   digitalWrite(trig, HIGH);
   delayMicroseconds(10);
   digitalWrite(trig, LOW);
   
   duration = pulseIn(echo, HIGH);

   cm = microsecondsToCentimeters(duration);

   Serial.print(cm);
   Serial.print("cm");
   Serial.println();
   delay(100);
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
