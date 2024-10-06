/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder EncX(2, 4);
//Encoder EncY(3, 19);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

//long oldPosition  = -999;

void loop() {
Serial.print("X: ");
Serial.println(EncX.read());
//Serial.print("\tY: ");
//Serial.println(EncY.read());
}
