import processing.net.*;
import controlP5.*;

ControlP5 view1, view2, view2_2, view3, view4;
Client client;
String distance1, speed1, data_type;
int speed_in, distance;
Boolean speedType;
Range range;
int speed;
Chart chartSpeed;
Chart chartDistance;
String data;
Textfield speedInput;
Textarea chartCaption_1;
Textarea chartCaption_2;

void setup() {
  size(600, 400);
  background(255, 255, 255);
  view1 = new ControlP5(this);
  client = new Client(this, "192.168.189.1", 5200);
  view1.addButton("Stop")
    .setValue(0)
    .setPosition(350, 100)
    .setSize(200, 50);

  view1.addButton("Start")
    .setValue(0)
    .setPosition(50, 100)
    .setSize(200, 50);

  view2 = new ControlP5(this);
  view2.addButton("Set_Speed")
    .setValue(0)
    .setPosition(50, 250)
    .setSize(200, 50);

  view2_2 = new ControlP5(this);
  view2_2.addButton("Cruise_Control")
    .setValue(0)
    .setPosition(350, 250)
    .setSize(200, 50);

  view2.hide();
  view2_2.hide();

  view3 = new ControlP5(this);
  speedInput = view3.addTextfield("Speed(cm/s)")
    .setPosition(350, 250)
    .setSize(200, 40)
    .setAutoClear(true);

  view3.hide();

  view4 = new ControlP5(this);
  
chartCaption_1 = view4.addTextarea("Current speed")
                .setPosition(50, 320)
                .setSize(200, 30)
                .setLineHeight(14)
                .setColorBackground(color(40))
                .setColorForeground(color(30, 50, 170))
                .scroll(1)
                .hideScrollbar()
                ;
chartCaption_2 = view4.addTextarea("Current distance to Object")
                .setPosition(350, 320)
                .setSize(200, 30)
                .setLineHeight(14)
                .setColorBackground(color(40))
                .setColorForeground(color(30, 50, 170))
                .scroll(1)
                .hideScrollbar()
                ;
view4.hide();
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
  //text("Data from Arduino:", 50, 250);
  
  // Request new data every second
  
  if (frameCount % 60 == 0) {
    data = client.readString();
  }
  
  
  //Arduino data on screen
  if (data != null) {
    //When the data it is not null print
    //text(data, 50, 280);
    println(data);
  }

}

void Stop(int theValue) {
  client.write("S");
  println("Stop");

  view2.hide();
  view2_2.hide();
  view3.hide();
  view4.hide();
}

void Start(int theValue) {
  client.write("G");

  println("Start");
  view2.show();
  view2_2.show();
  view3.hide();
  view4.hide();
}

void Set_Speed(int theValue) {
  client.write("U");
  view2_2.hide();
  view3.show();
}

void controlEvent(ControlEvent theEvent) {
  if (theEvent.isFrom(speedInput)) {
    String speedString = speedInput.getText();
    client.write(speedString);
    println("Speed sent: " + speedString);
  }
}



void Cruise_Control() {
  client.write("P");
  view3.hide();
  view4.show();
  view2.hide();
  view2_2.hide();
}
