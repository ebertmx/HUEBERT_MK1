/*
  Control program for BERT to read positional sensor data, average it and send it to
  BERT's control system
*/

#include <math.h>         // needed for the voltage2angle conversion math.
#include "Arduino.h"
#include "String.h"




#define pin_Q3 A0
#define pin_Q2 A1
#define pin_Q1 A2

#define Q1_maxAV 450
#define Q1_minAV 0

#define Q2_maxAV 1023
#define Q2_minAV 530

#define Q3_maxAV 1023
#define Q3_minAV 0

const double Q1_max = 2.41 ;
const double Q1_min = 0.33 ;
const double Q2_max = 2.79;
const double Q2_min = 0.52 ;
const double Q3_max = PI ;
const double Q3_min = -PI ;



#define button_pin 11



// Variable used to store the step targets of the HUE's stepper motors (alpha->1, beta->2, charlie->3)



//Timing variables
unsigned long previousMillis = millis();
unsigned long currentMillis = millis();
unsigned long time_interval = 10;


//------------------------------------------------------------------------------------------------

int readQvalue(String pinN);
bool checkbutton();

bool sending = false;
//-----------------------------------------------------



void setup()
{
  // Set serial to a baud rate of 9600 and configure analog and digital pins
  // Analog pins are for the potentiometers
  // Digital pin is for the control button



  Serial.begin(9600);
  pinMode(pin_Q1, INPUT);
  pinMode(pin_Q2, INPUT);
  pinMode(pin_Q3, INPUT);
 

  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);

  pinMode(12, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);

  digitalWrite(12, 0);

  digitalWrite(7, 1);
  digitalWrite(6, 0);
  digitalWrite(5, 1);
  digitalWrite(4, 0);
  digitalWrite(3, 1);
  digitalWrite(2, 0);

  Serial.println("BERT is Ready");
}


//-------------------------------------------------------------------------------------------------------------------------




//***********************************MAIN PROGRAM






void loop()
{


  // Instantiate local variables
  unsigned long previousMillis = millis();
  unsigned long currentMillis = millis();
  int n = 0;
  unsigned long int a = 0;
  unsigned long int b = 0;
  unsigned long int c = 0;


  bool button_pressed =  checkbutton();


  a = readQvalue(pin_Q1);

  b = readQvalue(pin_Q2);

  c = readQvalue(pin_Q3);

  if (button_pressed) {
    Serial.print("V");
    Serial.print(",");
    Serial.print(a);
    Serial.print(",");
    Serial.print(b);
    Serial.print(",");
    Serial.println(c);

    while(checkbutton())
    {};
  }

} //loop








//****-----------------------------------------------------


bool checkbutton() {
  unsigned int state = 0xff;
  int i = 0;
  while (i < 16) {
    i = i + 1;
    state = ((state << 1 | digitalRead(button_pin) | 0x00)) & 0xff;
  }
  if (0x00 == state) {
    return true;
  }
  else {
    return false;
  }


}

int readQvalue(const uint8_t pinN) {
  previousMillis = millis();
  currentMillis = millis();
  int n = 0;
  unsigned long int a = 0;
  // Average control values over period of time of interval
  while ((currentMillis - previousMillis) < time_interval)
  {
    a = analogRead(pinN) + a;
    n = n + 1;
    currentMillis = millis();
  }
  // Set control values to average over the time interval
  a = a / n;
  return a;
}


//------------------------------------------
