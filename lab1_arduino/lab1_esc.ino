#include <TimerOne.h>

unsigned pulseWidth = 1000; 
boolean calibrated = false;
boolean armed = false;
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(10, INPUT);
  Timer1.initialize(20000);
  Timer1.attachInterrupt(callbackPwm);
  digitalWrite(8, HIGH);
  digitalWrite(7, LOW);
  

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
    }
  if (digitalRead(10)==HIGH)
  {
    delay(50); //Debounce
    if (digitalRead(10)==HIGH)
      {
        armed = true;
      }
  }
  
  if (digitalRead(10)==LOW)
  {
    delay(50); //Debounce
    if (digitalRead(10)==LOW)
      {
        armed = false;
      }
  }
  
  if (armed)
  {
    pulseWidth = 200;
  }
  else
    pulseWidth = 1;
  
}
