#include <TimerOne.h>

unsigned pulseWidth = 1000; 
boolean calibrated = false;
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(9, OUTPUT);
  
  Timer1.initialize(20000);
  Timer1.attachInterrupt(callbackPwm);

}

void callbackPwm()
{
  digitalWrite(9, HIGH);
  delayMicroseconds(1000);
  delayMicroseconds(pulseWidth);
  digitalWrite(9, LOW);
}

// the loop function runs over and over again forever
void loop() {
  if (!calibrated)
  {
    delay(2000);
    pulseWidth = 1;
    delay(3000);
    calibrated = true;
    pulseWidth = 500;
  }
                    
}
