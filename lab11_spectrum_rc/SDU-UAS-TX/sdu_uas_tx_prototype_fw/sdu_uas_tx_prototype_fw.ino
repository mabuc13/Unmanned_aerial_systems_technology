/***************************************************************************
# SDU UAS Center TX firmware 
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# This library is based on the source example Generate_PPM_signal_V0.2 obtained
# from https://code.google.com/archive/p/generate-ppm-signal/downloads
#
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
This firmware is developed for the first prototype of the SDU UAS Transmitter

Revision
2018-04-16 KJ First released version
****************************************************************************/
/* parameters */

#define ppm_number 8  //set the number of ppm chanels
#define analog_number 5  //set the number of ppm chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10  //set PPM signal output pin on the arduino


/****************************************************************************/
/* variables */
/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[ppm_number];
float analog[analog_number];
short count;
boolean led_state;

/****************************************************************************/
void setup()
{  
  pinMode(LED_BUILTIN, OUTPUT); 

  Serial.begin(115200);

  //initiallize default ppm values
  for(int i=0; i<ppm_number; i++)
  {
    ppm[i]= default_servo_value;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}
/****************************************************************************/
ISR(TIMER1_COMPA_vect)
{
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state)
  {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else
  {  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= ppm_number)
    {
      digitalWrite(sigPin, !onState);

      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else
    {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
/****************************************************************************/
void loop()
{
  //put main code here
  static int val = 1;
  count ++;

  // update LED
  if (count % 10 == 0)
  {
    led_state = ! led_state;
    digitalWrite(LED_BUILTIN, led_state);
  }

  // read analog input
  analog[0] = analogRead(A2);
  analog[1] = analogRead(A3);
  analog[2] = analogRead(A0);
  analog[3] = analogRead(A1);
  analog[4] = analogRead(A4);

  ppm[0] = analog[0]*700/1023 + 1150 -6;
  if (ppm[0] > 2000)
    ppm[0] = 2000;
  ppm[1] =  analog[3]*700/1023 + 1150; // roll
  if (ppm[1] > 2000)
    ppm[1] = 2000;
  ppm[2] = (1023-analog[2])*700/1023 + 1150; // pitch
  if (ppm[2] > 2000)
    ppm[2] = 2000;
  ppm[3] = analog[1]*700/1023 + 1150;
  if (ppm[3] > 2000)
    ppm[3] = 2000;


  if (analog[4] > 500)
    ppm[5] = 1148;
  else
    ppm[5] = 1500; 

  if (analog[0] < 450 && analog[1] > 1000)
  {
    ppm[0] = 1150;
    ppm[3] = 1850;  
  }

    // output to serial port
  Serial.print (analog[0]);
  Serial.print (" ");
  Serial.print (analog[1]);
  Serial.print (" ");
  Serial.print (ppm[0]);
  Serial.print (" ");
  Serial.print (ppm[1]);
  Serial.print (" ");
  Serial.print (ppm[2]);
  Serial.print (" ");
  Serial.print (ppm[3]);
  Serial.print (" ");
  Serial.println (ppm[5]);

  delay(10);
}
/****************************************************************************/

