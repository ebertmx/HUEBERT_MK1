

//** This program handle interaction between the HUE system and a server providing serial data.
// Currently: Given a 20 byte serial string ended with a \n and consisting of letters and integers,
//  this code will convert the integer values into positional instruction for the Nema 17 stepper
//  motors contained in the HUE system.
//**Created By: Matthew Ebert and Jamieson Fregeau
//**Date: 2021-11-02

#include <AccelStepper.h> // library used for making the AccellStepper objects.
#include <math.h>         // needed for the voltage2angle conversion math.
#include "Arduino.h"
//These pins are connected from the arduino to the motor drivers.
#define MOTOR_X_ENABLE_PIN 8
#define MOTOR_X_STEP_PIN 3
#define MOTOR_X_DIR_PIN 2

#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 5
#define MOTOR_Y_DIR_PIN 4

#define MOTOR_Z_ENABLE_PIN 8
#define MOTOR_Z_STEP_PIN 7
#define MOTOR_Z_DIR_PIN 6

#define ENABLE_PIN 8

#define pin_UP 10
#define pin_DOWN 9
#define pin_RIGHT A3

#define pin_LEFT A2
#define pin_MIDDLE A4
#define pin_RST 11
#define pin_SET A5

#define pin_charlie_micro 12
#define pin_alpha_micro 13
#define pin_beta_micro 12

//#define Q_alpha_home 130
#define Q_alpha_home 509
//#define Q_beta_home 500
#define Q_beta_home 320
#define S_alpha_home_m 1015
#define S_beta_home_m 1013
#define S_alpha_home 86
#define S_beta_home 118

#define pin_Q2 A0
#define pin_Q1 A1
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

// Varialbes used to set the maximum speed and acceleration rates of HUE's stepper motors.
// Initial value set here
int alpha_speed = 350;
int beta_speed = 400;
int charlie_speed = 750;
int alpha_acc = 500;
int beta_acc = 500;
int charlie_acc = 1000;

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


//Stepper object definitions
AccelStepper alpha(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper beta(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper charlie(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);

// The string used to collect recieved serial input.
String s_input = "";

//check values
bool harderror = false;
bool position_copy = 0;
bool vector_copy = 0;
bool micro = false;
//functions
//------------------------------------------------------------------------------------------------
int Runvector();
int default_accel_vel();
int microOFF();
int microON();
int printbuttons();
bool outofbounds();
int runmotors();
void displaysensordata();
int homeHUE ();
int printmotorposition();
int voltage2angle(int a_in, int b_in, int c_in);
int printmotortarget();
int checkbuttons();
//-----------------------------------------------------





//-------------------------------------------------------------------------------------------------------------------------
void setup()
{
  //Runs once upon boot of arduino

  //------------------------Set up steppers motors
  alpha.setEnablePin(MOTOR_X_ENABLE_PIN);
  alpha.setPinsInverted(true, false, false);
  beta.setEnablePin(MOTOR_Y_ENABLE_PIN);
  beta.setPinsInverted(false, false, false);
  charlie.setEnablePin(MOTOR_Z_ENABLE_PIN);
  charlie.setPinsInverted(false, false, false);

  alpha.setAcceleration(alpha_acc);
  beta.setAcceleration(beta_acc);
  charlie.setAcceleration(charlie_acc);

  alpha.enableOutputs();
  beta.enableOutputs();
  charlie.enableOutputs();

  alpha.setMaxSpeed(alpha_speed);
  beta.setMaxSpeed(beta_speed);
  charlie.setMaxSpeed(charlie_speed);

  //---------------------------
  //-----------------------Serial set up
  Serial.begin(9600);
  Serial.flush();
  Serial.setTimeout(100);
  s_input.reserve(20); //Reserve data up to 10 character or integers


  //define pin modes

  //These are used by stepper objects
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(pin_alpha_micro, OUTPUT);
  pinMode(pin_beta_micro, OUTPUT);
  pinMode(pin_charlie_micro, OUTPUT);

  //These are used by sensors
  pinMode(pin_UP, INPUT_PULLUP);
  pinMode(pin_DOWN, INPUT_PULLUP);
  pinMode(pin_LEFT, INPUT_PULLUP);
  pinMode(pin_RIGHT, INPUT_PULLUP);
  pinMode(pin_MIDDLE, INPUT_PULLUP);
  pinMode(pin_RST, INPUT_PULLUP);
  pinMode(pin_SET, INPUT_PULLUP);



  //These are extra ihputs
  pinMode(pin_Q1, INPUT);
  pinMode(pin_Q2, INPUT);

  // This enables the stepper motors. Set pin 8 high to disable
  digitalWrite(ENABLE_PIN, LOW);

  //home motors
  //Set motors to home position: Make sure HUE starts from this defined position
  Serial.println("HUE--initiated ");
  microON();
  homeHUE();

  Serial.println("HUE--standing by; ");
  //Serial.println("R");
  //-----------------------------
} //setup




//***********************************MAIN PROGRAM






void loop()
{
  // a check variable used for return values of functions
  //read Serial Data if available in a string


  if (Serial.available())
  {
    //Serial.println("reading serial...");
    //s_input = Serial.readStringUntil('\n');

    //parse the input string if it has content
    //call functions and tasks here based on the first character in string
    char type = Serial.read();
    if (type == 'p')
    {
      S1 = Serial.readStringUntil('a').toInt();
      S2 = Serial.readStringUntil('b').toInt();
      S3 = Serial.readStringUntil('c').toInt();
      spd = Serial.readStringUntil('s').toInt();
      //Serial.readStringUntil('\n');

      //if the positional data was valid and converted to step value successfully set targets for steppers.
      // Serial.println("HUE--setting destination...");
      alpha.moveTo(S1);
      beta.moveTo(S2);
      charlie.moveTo(S3);
      alpha.setMaxSpeed(spd * 30);
      beta.setMaxSpeed(spd * 35);
      charlie.setMaxSpeed(spd * 70);

      Serial.println("HUE--running actuators (position)");
      runmotors();
    } // if (p)


    if (type == 'v') {
      int len = Serial.readStringUntil('L').toInt();
      int Stepvec[len][4];
      for (int counter = 0; counter < len; counter++) {
        while (!Serial.available()) {
          delay(1);
        }
        Stepvec[counter][0] = Serial.readStringUntil('a').toInt();
        Stepvec[counter][1] = Serial.readStringUntil('b').toInt();
        Stepvec[counter][2] = Serial.readStringUntil('c').toInt();
        Stepvec[counter][3] = Serial.readStringUntil('s').toInt();
      }
      Serial.println("HUE--running actuators (vector)");
      spd = Stepvec[0][3];
      alpha.setMaxSpeed(spd * 30);
      beta.setMaxSpeed(spd * 35);
      charlie.setMaxSpeed(spd * 70);

      for ( int x = 0; x < len; x++) {
        //Serial.println(x);
        S1 = Stepvec[x][0];
        S2 = Stepvec[x][1];
        S3 = Stepvec[x][2];

        alpha.moveTo(S1);
        beta.moveTo(S2);
        charlie.moveTo(S3);
        //Serial.println("HUE--running actuators (vector)");
        runmotors();
        //printmotorposition();
      }


    }//if v


    else if (type == 'k') {
      printmotorposition();
    }
    else if (type == 's') {
      digitalWrite(ENABLE_PIN, HIGH);
      Serial.print("type 'a' to activate motors or reset");
    }
    else if (type == 'a') {
      digitalWrite(8, LOW);
    }
    else if (type == 'm') {
      microON();
      Serial.print("type 'f' to disable micro stepping");
    }
    else if (type == 'f') {
      //microOFF();
      Serial.print("type 'f' to disable micro stepping");
    }

    //end of else if
    Serial.flush();
    default_accel_vel();
    printmotorposition();
    Serial.println("HUE--Standing By");
    Serial.println("E");
  }   //while

  checkbuttons();
} //loop








//****-----------------------------------------------------


int runmotors() {
  //  if (!micro) {
  //    while ((alpha.isRunning() || beta.isRunning() || charlie.isRunning()) )
  //    {
  //      alpha.run();
  //      beta.run();
  //      charlie.run();
  //      //      if (outofbounds()) {
  //      //        Serial.println("HUEBERT IS OUT OF BOUNDS!!");
  //      //        homeHUE();
  //      //        break;
  //      //      }
  //    }//while
  //  }
  //
  //  if (micro) {
  // Serial.println("Running Actuators");
  while ((alpha.isRunning() || beta.isRunning() || charlie.isRunning()) )
  {
    alpha.run();
    beta.run();
    charlie.run();
    //    if (outofbounds()) {
    //      Serial.println("HUEBERT IS OUT OF BOUNDS!!");
    //      homeHUE();
    //    }
  }//while
  //}
}





int homeHUE ()
{
  Serial.println("Setting HUEBERT to Home Position");
  default_accel_vel();

  int alpha_dir = 1;
  int alpha_angle = readQ1();
  int temp = 0;
  int test = 0;
  int alpha_diff = Q_alpha_home - alpha_angle;

  if (alpha_angle > Q_alpha_home) {
    alpha_dir = 1;
  }
  else {
    alpha_dir = -1;
  }

  int segment = 2;
  while (abs(alpha_diff) >= 1) {
    int m = alpha_diff /segment - alpha_dir;
//    Serial.print("m: ");
//    Serial.println(m);
    alpha.move(m);

    while (alpha.isRunning()  &&  (AV_Q1_max > analogRead(pin_Q1) > 0)) {
      alpha.run();
      beta.run();
      if (analogRead(pin_Q2) > (Q_beta_home - 10)) {
        beta.move(-10);
      }
      if (analogRead(pin_Q2) < 100) {
        beta.move(+20);
      }

    }
    temp = alpha_diff;
    alpha_angle = readQ1();
    alpha_diff = Q_alpha_home - alpha_angle;
//    Serial.println("alpha_angle");
//    Serial.println(alpha_angle);
//    Serial.println("alpha_diff");
//    Serial.println(alpha_diff);
    if (temp == alpha_diff) {
      test++;
//      delay(10);
    } else {
      test = 0;
    }
    if (test > 10) {
      Serial.println("HUEBERT IS BLOCKED!!");
      break;
    }
  }//alpha home

  if (test > 10) {
    return -1;
  }
  int beta_angle = readQ2();
  int beta_diff = Q_beta_home - beta_angle;
  test = 0 ;

  while (beta_diff >= 1) {
    beta.move(beta_diff / segment  + 1);
    while (beta.isRunning()  &&   (AV_Q2_max > analogRead(pin_Q2) > 0)) {
      beta.run();
    }
    temp = beta_diff;
    beta_diff = Q_beta_home - readQ2();
//    Serial.println("beta_angle");
//    Serial.println(beta_angle);
//    Serial.println("beta_diff");
//    Serial.println(beta_diff);
    //    delay(1000);
    if (temp == beta_diff) {
      test++;
//      delay(50);
    } else {
      test = 0;
    }

    if (test > 10) {
      Serial.println("HUEBERT IS BLOCKED!!");
      break;
    }
    // Serial.println(beta_diff);
  }//beta home

  // if (micro) {
  alpha.setCurrentPosition(S_alpha_home_m);
  beta.setCurrentPosition(S_beta_home_m);
  charlie.setCurrentPosition(0);
  //  } else if (!micro) {
  //    alpha.setCurrentPosition(S_alpha_home);
  //    beta.setCurrentPosition(S_beta_home);
  //    charlie.setCurrentPosition(0);
  //  }
  printmotorposition();
//  Serial.println("HUE--Standing By");
//  Serial.println("E");
}


int readQ2() {
  previousMillis = millis();
  currentMillis = millis();
  time_interval = 5;
  int n = 0;
  unsigned long int a = 0;
  // Average control values over period of time of interval
  while ((currentMillis - previousMillis) < time_interval)
  {
    a = analogRead(A0) + a;
    n = n + 1;
    currentMillis = millis();
  }
  // Set control values to average over the time interval
  a = a / n;
  return a;
}

int readQ1() {
  previousMillis = millis();
  currentMillis = millis();
  time_interval = 5;
  int n = 0;
  unsigned long int a = 0;
  // Average control values over period of time of interval
  while ((currentMillis - previousMillis) < time_interval)
  {
    a = analogRead(pin_Q1) + a;
    n = n + 1;
    currentMillis = millis();
  }
  // Set control values to average over the time interval
  a = a / n;
  return a;
}


int printmotorposition() {
  Serial.print("HUE--at position:  ");
  Serial.print("p");
  Serial.print(alpha.currentPosition());
  Serial.print("a");
  Serial.print(beta.currentPosition());
  Serial.print("b");
  Serial.print(charlie.currentPosition());
  Serial.println("c");
}

int printmotortarget() {
  Serial.print("HUE--targeting position:  ");
  Serial.print("p");
  Serial.print(alpha.targetPosition());
  Serial.print("a");
  Serial.print(beta.targetPosition());
  Serial.print("b");
  Serial.print(charlie.targetPosition());
  Serial.println("c");
}


bool outofbounds() {
  //  alpha_angle_f = analogRead(pin_Q1);
  //  beta_angle_f = analogRead(pin_Q2);
  if (AV_Q2_max < analogRead(pin_Q2) < 1) {
    return false;
  }
  if (AV_Q1_max < analogRead(pin_Q1) < 1) {
    return false;
  }
}


int checkbuttons() {
  default_accel_vel();
  int input_pins = B10000000 + ((PINC & B111100) << 1) + (((PINB & B01110) >> 1));

  switch (input_pins) {
    case B11111101:
      // Serial.println("UP");
      alpha.move(1000);

      break;
    case B11111110:
      // Serial.println("DOWN");
      alpha.move(-1000);
      break;
    case B11110111:
      // Serial.println("LEFT");
      charlie.move(1000);

      break;
    case B11101111:
      // Serial.println("RIGHT");
      charlie.move(-1000);

      break;
    case B11011111:
      // Serial.println("MIDDLE");
      delay(500);
      input_pins = input_pins = B10000000 + ((PINC & B111100) << 1) + (((PINB & B01110) >> 1));
      if (input_pins == B11011111) {
        homeHUE();
      }
      break;
    case B11111011:
      //Serial.println("SET");
      beta.move(1000);
      break;
    case B10111111:
      //Serial.println("RST");
      beta.move(-1000);
      break;
  }

  int input_next = B10000000 + ((PINC & B111100) << 1) + (((PINB & B01110) >> 1));
  while (input_next == input_pins && input_next != B11111111) {
    alpha.run();
    beta.run();
    charlie.run();
    input_next = B10000000 + ((PINC & B111100) << 1) + (((PINB & B01110) >> 1));
  }
  alpha.stop();
  beta.stop();
  charlie.stop();
  runmotors();
}//checkbuttons

int default_accel_vel() {
  microON();
  alpha.setAcceleration(alpha_acc);
  beta.setAcceleration(beta_acc);
  charlie.setAcceleration(charlie_acc);

  alpha.setMaxSpeed(alpha_speed);
  beta.setMaxSpeed(beta_speed);
  charlie.setMaxSpeed(charlie_speed);
}

int microON() {
  //Serial.println("Micro ON");
  digitalWrite(pin_alpha_micro, HIGH);
  digitalWrite(pin_beta_micro, HIGH);
  digitalWrite(pin_charlie_micro, HIGH);
  micro = true;
  return 1;
}

int microOFF() {
  //Serial.println("Micro OFF");
  digitalWrite(pin_alpha_micro, LOW);
  digitalWrite(pin_beta_micro, LOW);
  digitalWrite(pin_charlie_micro, LOW);
  micro = false;
  return -1;
}


int printbuttons() {
  Serial.print("UP: ");
  Serial.println(digitalRead(pin_UP));
  Serial.print("DOWN: ");
  Serial.println(digitalRead(pin_DOWN));
  Serial.print("RST: ");
  Serial.println(digitalRead(pin_RST));
  Serial.print("SET: ");
  Serial.println(digitalRead(pin_SET));
  Serial.print("RIGHT: ");
  Serial.println(digitalRead(pin_RIGHT));
  Serial.print("LEFT: ");
  Serial.println(digitalRead(pin_LEFT));
  Serial.print("MIDDLE: ");
  Serial.println(digitalRead(pin_MIDDLE));
  //delay(100);
}


//------------------------------------------
