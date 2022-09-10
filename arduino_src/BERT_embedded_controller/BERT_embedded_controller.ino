/*
  Control program for BERT to read positional sensor data, average it and send it to
  BERT's control system
*/

#include <math.h>         // needed for the voltage2angle conversion math.
#include "Arduino.h"
#include "String.h"
//These pins are connected from the arduino to the motor drivers.


//#define Q_alpha_home 130
#define Q_alpha_home 509
//#define Q_beta_home 500
#define Q_beta_home 320
#define S_alpha_home_m 1015
#define S_beta_home_m 1013
#define S_alpha_home 86
#define S_beta_home 118

#define pin_Q3 A7
#define pin_Q2 A6
#define pin_Q1 A5

#define button 2

// These define the maximum relative angles which the HUE frame can take (in radians)
//  multiplied by the precision factor and stored as integers.
const double Q1_max = 2.41 ;
const double Q1_min = 0.33 ;
const double Q2_max = 2.79;
const double Q2_min = 0.52 ;
const double Q3_max = PI ;
const double Q3_min = -PI ;
const double offset1 = 0.33272;
const double offset2 = 1.84818;
const double offset3 = 0.88631;

const unsigned int AV_Q1_max = 900;
const unsigned int AV_Q2_max = 600;
const unsigned int AV_Q3_max = 1000;

const int p = 1000;
// Constants for voltage2angle.
// These value define HUE and BERTS mechanical frame dimensions
const  int OG = 0.0800 * p; //m*p
const  int L1 = 0.255 * p; //m
const  int CL1 = 0.14142136 * p; //m
const  int L1m = 0.1100 * p; //m
const  int L2 = 0.2600 * p; //m
const  int CL2 = 0.28044607 * p; //m
const int L2e = 0.0800 * p; //m
const  int OL = 0.109337002 * p; //m
const  int OX = 0.1055 * p; //m



// Variable used to store the step targets of the HUE's stepper motors (alpha->1, beta->2, charlie->3)

int S1;
int S2;
int S3;
int spd;
int x_current;
int y_current;
int z_current;

//Timing variables
unsigned long previousMillis = millis();
unsigned long currentMillis = millis();
unsigned long time_interval = 10;


//------------------------------------------------------------------------------------------------

int readQvalue(String pinN);
bool outofbounds();
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
  pinMode(button, INPUT);

  pinMode(3, OUTPUT);
  digitalWrite(3, 1);
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



  a = readQvalue(pin_Q1);

  b = readQvalue(pin_Q2);

  c = readQvalue(pin_Q3);

  // Reset time
  // previousMillis = currentMillis;
  Serial.print("\nc = ");
  Serial.println(c);
//  Serial.print("\nb = ");
//  Serial.println(b);
//  Serial.print("\nc = ");
//  Serial.print(c);
  delay(10);


} //loop








//****-----------------------------------------------------




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


bool outofbounds() {

  if (AV_Q2_max < analogRead(pin_Q2) < 1) {
    return false;
  }
  if (AV_Q1_max < analogRead(pin_Q1) < 1) {
    return false;
  }
  if (AV_Q1_max < analogRead(pin_Q3) < 1) {
    return false;
  }
}


//------------------------------------------
