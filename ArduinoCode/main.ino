#include <Quadcopter.h>

Quadcopter copter;

void setup() {
   Serial.begin(115200);
   Wire.begin();
   ss.begin(4800); //for gps including baud rate

   copter.Initialisation.InitialiseSystem();

}

void loop() {
  copter.SystemStates.Run();
}
