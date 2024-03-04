import processing.net.*;
Client myClient;
String data;
//int Object_distance;


void setup() {
  //First change the IP address to match Arduino
myClient = new Client(this,"IP",5200); 
 
 size(600, 400);
}
void draw() {
  //green square formatting
fill(0, 200, 0);
rect(0, 0, 300, 400);
fill(255, 255, 255);
textSize(60);
text("Start", 80, 200);

//red square formatting
fill(200, 0, 0);
rect(300, 0, 600, 400);
fill(255, 255, 255);
textSize(60);
text("Stop", 380, 200);

 //inital setup recieve to make sure it is working 
data = myClient.readString();
if(data != null){
println(data);
}

//get info from ultrasonics
//will change to show up something here
//have to decide if we are sending a distance or just the fact there is something
//Object_distance = myClient.read();
//println(Object_distance);



 
 

} 
//function for when it is clicked
void mouseClicked() {
  //first get postion of the mouse
  if (0 < mouseX & mouseX< 300){ //if the mouse is on the green part
    myClient.write("G");
    println("Starting"); //to troubleshoot it is working
  //  for (int i = 0; i<=10; i++){ //sends the 1's 12 times total, should be enough??
    //myClient.write(1);
    //}
  }
  else if (300< mouseX & mouseX<600){//if mouse is on red part
  myClient.write("S");
  println("Stopping");
    //for (int i = 0; i<=10; i++){ //sends the 0's 12 times total
    //myClient.write(0);
   //}
}
}
