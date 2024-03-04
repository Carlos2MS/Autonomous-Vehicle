import processing.net.*;
import controlP5.*;


ControlP5 p5;
Client client;
String data;


void setup() {
  size(600, 400);
  background(255, 255, 255);
  p5 = new ControlP5(this);
  client = new Client(this, "IP", 5200);
    p5.addButton("Stop")
    .setValue(0)
    .setPosition(350, 100)
    .setSize(200, 50);
  
  p5.addButton("Start")
    .setValue(0)
    .setPosition(50, 100)
    .setSize(200, 50);


}

void draw() {
  if (frameCount % 60 == 0) {
    background(255, 255, 255);
  }
  
  //Texts on Screen
  textSize(30);
  fill(0, 0, 0);
  text("Wall-E", 260,50);
  textSize(20);
  fill(0, 0, 0);
  text("Data from Arduino:", 50, 250);
  
  // Request new data every second
  
  if (frameCount % 60 == 0) {
    data = client.readString();
  }
  
  
  //Arduino data on screen
  if (data != null) {
    //When the data it is not null print
    text(data, 50, 280);
    println(data);
  }
  
}

void Stop(int theValue) {
  client.write("S");
}


void Start(int theValue) {
  
  client.write("G");
}
