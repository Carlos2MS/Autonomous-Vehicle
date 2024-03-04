// IR Sensor Pins
const int REYE = 10;
const int LEYE = 8;

void setup() {
  Serial.begin(9600);
  pinMode( LEYE, INPUT );
  pinMode( REYE, INPUT );
}

void loop() {
  
  //Task 1
  //Low=black
  //White =high

  if( digitalRead( LEYE ) == HIGH ){
    Serial.print("low  ");
  }else{
    Serial.print("high ");
  }

  if( digitalRead( REYE ) == HIGH ){
    Serial.println("low  ");
  }else{
    Serial.println("high ");
  }

  //Task 2
  /*
  bool all_clearL = digitalRead( LEYE );
  bool all_clearR = digitalRead( REYE );

  if( !all_clearL || !all_clearR ){ //!all_clear =black
  Serial.println("I see you!");
  }else{
  Serial.println("No");
  }
  delay( 1000 );
  
  */
}