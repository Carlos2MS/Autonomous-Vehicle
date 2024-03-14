#include "Arduino_LED_Matrix.h"
//Web page to make the matrix,draws
//https://ledmatrix-editor.arduino.cc/

ArduinoLEDMatrix matrix;

void setup() {
  Serial.begin(115200);
  matrix.begin();
}

const uint32_t happy[] = {
    0x19819,
    0x80000001,
    0x81f8000
};
const uint32_t heart[] = {
    0x3184a444,
    0x44042081,
    0x100a0040
};
const uint32_t straight[] = {
		0x6006006,
		0x602641,
		0x680f0060,
		66
};

const uint32_t turnL[] = {
		0x18018019,
		0x1881fc1,
		0xfc008010,
		66
};

const uint32_t turnR[] = {
		0xc00c04,
		0xc08c1fc1,
		0xfc080040,
		66
};

const uint32_t stop[] = {
		0xf010828,
		0x42442242,
		0x141080f0,
		66
};


void loop(){
  matrix.loadFrame(stop);
  delay(100);
}
