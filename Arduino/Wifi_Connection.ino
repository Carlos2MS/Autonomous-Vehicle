//https://docs.arduino.cc/tutorials/uno-wifi-rev2/uno-wifi-r2-hosting-a-webserver/

#include <WiFiS3.h>
char ssid[] = "x";
char pass[] = "x";

WiFiServer server(5200);
WiFiClient client = server.available();

// IR PINS
//const int IRPin = A0;

void setup() {
 Serial.begin(9600);
 WiFi.begin(ssid, pass);
 IPAddress ip = WiFi.localIP();
 Serial.print("IP Address:");
 Serial.println(ip);
 server.begin();
 //pinMode( IRPin, INPUT );

}

void loop() {
 //int voltage = analogRead(IRPin);
 client = server.available();

 if (client.connected()) {
 client.write("Hello Client");
 } 
}
