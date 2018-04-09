#include <TimerOne.h>

unsigned pulseWidth = 1; 
boolean calibrated = false;
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  Timer1.initialize(20000);
  Timer1.attachInterrupt(callbackPwm);
  digitalWrite(8, HIGH);
  digitalWrite(7, LOW);

}

void callbackPwm()
{
  digitalWrite(9, HIGH);
  delayMicroseconds(400);
  delayMicroseconds(pulseWidth);
  digitalWrite(9, LOW);
}

// the loop function runs over and over again forever
void loop() {

 positionMotor(0);
 delay(1500);
 positionMotor(180);
 delay(1500);
  
 

  
  
 //Max pulswidth = 2350;
 //min pulsewidth = 400
 //Rækkevidde 1950
 //center = 1350
}

void positionMotor(int deg)
{
 pulseWidth = deg*10.83;
 if ( pulseWidth == 0)
     pulseWidth = 1;
 Serial.print(deg);
 Serial.print("  ");
 Serial.print(pulseWidth);
 Serial.print("\n");
}
