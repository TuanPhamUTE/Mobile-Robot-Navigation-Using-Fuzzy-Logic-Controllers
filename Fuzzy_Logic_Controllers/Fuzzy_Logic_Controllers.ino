#include <math.h>        
//***** Import SoftwareSerial library for UART (Driver) *****//
#include <SoftwareSerial.h>
SoftwareSerial mySerial(17, 16); //Define PIN17 & PIN16 as software

//***** Import Fuzzy library *****//
#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzyOutput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzySet.h>

//***** Encoder Pin *****//
const int encoderPinRA = 2;
const int encoderPinRB = 3;
const int encoderPinLA = 20;
const int encoderPinLB = 21;

//***** Robot's Parameter Variable *****//
float B = 13.5;
float R = 7.25;
float D = 14.5; // 2R
float L = 27.0; // 2B
#define PI 3.1415926535897932384626433832795

//***** Odometry Variable *****//
float phi_degree = 0;
float phi_rad = 0;
float distance_left = 0, distance_right = 0, distance_mid = 0;
float x = 0, y = 0, phi = 0, distance_traveled;
volatile long int pulseL = 0;
volatile long int pulseR = 0;
int pre_pulse_left = 0, pre_pulse_right = 0;
int delta_pulse_left = 0, delta_pulse_right = 0;

float Xd = 300;
float Yd = 180;
float phi_target = atan2(Yd, Xd);
float phi_temp = 0;
float d_target = sqrt(pow(Xd,2) + pow(Yd,2));
int d_actual = 0;
int phi_actual = 0;

float d0 = d_target*0.10;
float d1 = d_target*0.25;
float d2 = d_target*0.38;
float d3 = d_target*0.50;
float d4 = d_target*0.63;
float d5 = d_target*0.75;
float d6 = d_target*0.88;
float d7 = d_target*0.110;

//***** Timer Variable *****//
int time_delay = 50;
volatile unsigned present_time = 0;
volatile unsigned previous_time = 0;
volatile unsigned delta_time = 0;

//***** Config Pin of Ultrasonics Sensor *****//
const int trig1 = 53;      
const int trig2 = 51;      
const int trig3 = 49;      
const int trig4 = 47;     
const int trig5 = 45;     
const int trig6 = 43;     
const int trig7 = 41;
     
const int echo1 = 24;    
const int echo2 = 25;     
const int echo3 = 26;     
const int echo4 = 27;     
const int echo5 = 28;     
const int echo6 = 29;    
const int echo7 = 30; 

const int echo_behind1 = 8;    
const int echo_behind2 = 10; 
const int trig_behind1 = 9;    
const int trig_behind2 = 11; 

int fil1 = 0;
int fil2 = 0;

//***** Variable of distance from ultrasonic *****//
unsigned long duration;
int distance;
int distance1;           
int distance2;           
int distance3;           
int distance4;          
int distance5;           
int distance6;           
int distance7;
int distance_behind1;           
int distance_behind2;
int DLeft = 0, DCenter = 0, DRight = 0;

//***** Bluetooth *****//
int ctrlMode = 0;
int baudRate[] = {300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
char value;

//***** DOCKING *****//
String data_docking;
float dock_x1;
float dock_z1;

//***** DANCING *****//
String dancing_var = "";
int state = 0;
int setHome = 0;

//***** Resert Value *****//
void resert_value(){
  phi_degree = 0;
  distance_left = 0;
  distance_right = 0;
  distance_mid = 0;
  x = 0;
  y = 0;
  phi = 0;
  distance_traveled;
  pulseL = 0;
  pulseR = 0;
  pre_pulse_left = 0;
  pre_pulse_right = 0;
  delta_pulse_left = 0;
  delta_pulse_right = 0;
}

//***** Odometry *****//
void odometry()
{ 
    delta_pulse_left = pulseL - pre_pulse_left;
    distance_left = PI*D*(delta_pulse_left/(double)1600);
    
    delta_pulse_right = pulseR - pre_pulse_right;
    distance_right = PI*D*(delta_pulse_right/(double)1600);
    
    distance_mid = (distance_left + distance_right)/2;
    
    x = x + distance_mid*cos(phi);
    y = y + distance_mid*sin(phi);
    phi = phi + ((distance_right - distance_left)/L);
    phi = atan2(sin(phi), cos(phi));
    phi_degree = (phi*180)/PI;
    
//    Serial.print("x: "); Serial.println(x);
//    Serial.print("y: "); Serial.println(y);
//    Serial.print("phi"); Serial.println(phi_degree);
    Serial.print(x); Serial.print(","); Serial.print(y); Serial.print(","); Serial.println(phi_degree);

    pre_pulse_left = pulseL;
    pre_pulse_right = pulseR;
}

//****** Find Near *****//
int findNear(int dist1, int dist2, int dist3){
  int minVal = dist1;
  if(dist2<dist1){
    minVal = dist2;
  }
  if(dist3<dist2 && dist3<dist1){
    minVal = dist3;
  }
  return minVal;
}

//***** Calculate the distances from ultrasonic sensors *****//
int distance_func(int trig, int echo)
{    
  //***** Creat Pulse for Trig Pin *****//
  digitalWrite(trig,LOW);   
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);  
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  //***** Calculate Distance *****//
  duration = pulseIn(echo,HIGH,10000);    //đo độ rộng xung HIGH ở chân echo.
//  distance = int((duration/2/29.412)+ 1); //tính khoảng cách đến vật.

  // Calculate the distance:
  distance = duration * 0.034 / 2;

  if (distance==0){
    distance = 100;
  }      
  if(distance >= 10){
    fil1 = distance;    
  }
  if(distance >= 10){
    fil2 = distance;
  }
  if(distance < 10){
    fil2 = fil1;
  }
  return fil2;
} 

//***** Motor control function for robot *****// (N1 Trái, N2 Phải)
//***** Turn Right *****//
void Turn_Right(){ 
  mySerial.println("{N2 P0 V-100 A200}");delay(2);mySerial.println("{N1 P0 V-100 A200}");
}

//***** Turn Left *****//
void Turn_Left(){
  mySerial.println("{N2 P0 V100 A200}");delay(2);mySerial.println("{N1 P0 V100 A200}");
}

//***** Forward *****//
void Forward(int LeftSpeed, int RightSpeed){
//  mySerial.println("{N2 P0 V502.6 A500}");delay(2);mySerial.println("{N1 P0 V-501 A500}"); // N1 trai, N2 Phai

  mySerial.println("{N1 P0 V"+String(-LeftSpeed)+" A600}"); delay(2); mySerial.println("{N2 P0 V"+String(RightSpeed)+" A600}");
}

void Backward(int LeftSpeed, int RightSpeed){
//  mySerial.println("{N2 P0 V502.6 A500}");delay(2);mySerial.println("{N1 P0 V-501 A500}"); // N1 trai, N2 Phai

  mySerial.println("{N2 P0 V"+String(-LeftSpeed)+" A600}"); delay(2); mySerial.println("{N1 P0 V"+String(RightSpeed)+" A600}");
}

//***** Stop *****//
void Stop(){
  mySerial.println("{N2 P0 V0 A500}");delay(2);mySerial.println("{N1 P0 V0 A500}");
}

void Forward_Right(){
  mySerial.println("{N2 P0 V100 A600}");delay(2);mySerial.println("{N1 P0 V-200 A600}");
}

void Forward_Left(){
  mySerial.println("{N2 P0 V200 A600}");delay(2);mySerial.println("{N1 P0 V-100 A600}");
}

void Backward_Left(){
  mySerial.println("{N2 P0 V-200 A600}");delay(2);mySerial.println("{N1 P0 V100 A600}");
}

void Backward_Right(){
  mySerial.println("{N2 P0 V-100 A600}");delay(2);mySerial.println("{N1 P0 V200 A600}");
}

void Read_Bluetooth(){
  //***** Code Bluetooth *****//
  if (Serial3.available()>0) {
    state = 0;
    setHome= 0;
    value = Serial3.read();
    switch(value){
      case 'M':
        ctrlMode = 1;
        break;
      case 'U':
        ctrlMode = 2;
        break;
      case 'O':
        ctrlMode = 3;
        break;
    }
  }
}

//***** SET UP FUZZY *****//
Fuzzy *fuzzy_obstacle = new Fuzzy();
Fuzzy *fuzzy_tracking = new Fuzzy();
Fuzzy *fuzzy_docking = new Fuzzy();

//************************* DOCKING *************************//
//***** fuzzy sets *****//
//***** X *****// (Left)
FuzzySet *dock_XBN = new FuzzySet(-40,-40,-25,-15);
FuzzySet *dock_XN = new FuzzySet(-25,-15,-15,0);
FuzzySet *dock_XZ = new FuzzySet(-0.8,0,0,0.8);
FuzzySet *dock_XP = new FuzzySet(0,15,15,25);
FuzzySet *dock_XBP = new FuzzySet(15,25,40,40);

//***** Z *****// (Center)
FuzzySet *dock_ZZ = new FuzzySet(5,5,25,50);
FuzzySet *dock_ZN = new FuzzySet(25,50,50,75);
FuzzySet *dock_ZM = new FuzzySet(50,75,75,100);
FuzzySet *dock_ZF = new FuzzySet(75,100,190,190);

//***** Motor 1 *****// (Left)
FuzzySet *dock_leftBB = new FuzzySet(-155, -155, -95, -55);
FuzzySet *dock_leftBS = new FuzzySet(-95, -55, -55, 0);
FuzzySet *dock_leftZ = new FuzzySet(-15, 0, 0, 15);
FuzzySet *dock_leftFS = new FuzzySet(0, 55, 55, 95);
FuzzySet *dock_leftFB = new FuzzySet(55, 95, 155, 155);

//***** Motor 2 *****// (Right)
FuzzySet *dock_rightBB = new FuzzySet(-155, -155, -95, -55);
FuzzySet *dock_rightBS = new FuzzySet(-95, -55, -55, -10);
FuzzySet *dock_rightZ = new FuzzySet(-15, 0, 0, 15);
FuzzySet *dock_rightFS = new FuzzySet(10, 55, 55, 95);
FuzzySet *dock_rightFB = new FuzzySet(55, 95, 155, 155);


//************************* OBSTACLE AVOIDANCE*************************//
//***** fuzzy sets *****//
//***** Sensor 1 *****// (Left)
FuzzySet *near1 = new FuzzySet(0,10,20,30);
FuzzySet *mid1 = new FuzzySet(20,30,40,50);
FuzzySet *far1 = new FuzzySet(40,50,20000,20000);

//***** Sensor 2 *****// (Center)
FuzzySet *near2 = new FuzzySet(0,10,20,30);
FuzzySet *mid2 = new FuzzySet(20,30,40,50);
FuzzySet *far2 = new FuzzySet(40,50,20000,20000);

//***** Sensor 3 *****// (Right)
FuzzySet *near3 = new FuzzySet(0,10,20,30);
FuzzySet *mid3 = new FuzzySet(20,30,40,50);
FuzzySet *far3 = new FuzzySet(40,50,20000,20000);

//***** Motor 1 *****// (Left)
FuzzySet *leftBB = new FuzzySet(-160, -160, -95, -55);
FuzzySet *leftBS = new FuzzySet(-95, -55, -55, -5);
FuzzySet *leftZ = new FuzzySet(-5, 0, 0, 5);
FuzzySet *leftFS = new FuzzySet(5, 55, 55, 95);
FuzzySet *leftFB = new FuzzySet(55, 95, 160, 160);

//***** Motor 2 *****// (Right)
FuzzySet *rightBB = new FuzzySet(-160, -160, -95, -55);
FuzzySet *rightBS = new FuzzySet(-95, -55, -55, -5);
FuzzySet *rightZ = new FuzzySet(-5, 0, 0, 5);
FuzzySet *rightFS = new FuzzySet(5, 55, 55, 95);
FuzzySet *rightFB = new FuzzySet(55, 95, 160, 160);

//************************* TRACKING *************************//
//***** Distance *****//
//FuzzySet *dist_Z =  new FuzzySet(-2,-2,0,d0); //d_target
//FuzzySet *dist_N =  new FuzzySet(d0, d1, d1, d2);
//FuzzySet *dist_VN = new FuzzySet(d1, d2, d2, d3);
//FuzzySet *dist_M =  new FuzzySet(d2, d3, d3, d4);
//FuzzySet *dist_SF = new FuzzySet(d3, d4, d4, d5);
//FuzzySet *dist_F =  new FuzzySet(d4, d5, d5, d6);
//FuzzySet *dist_VF = new FuzzySet(d5, d6, d7, d7);
//
FuzzySet *dist_Z =  new FuzzySet(d5, d6, d7, d7);   //d_target
FuzzySet *dist_VN =  new FuzzySet(d4, d5, d5, d6);
FuzzySet *dist_N = new FuzzySet(d3, d4, d4, d5);
FuzzySet *dist_M =  new FuzzySet(d2, d3, d3, d4);
FuzzySet *dist_SF = new FuzzySet(d1, d2, d2, d3);
FuzzySet *dist_F =  new FuzzySet(d0, d1, d1, d2);
FuzzySet *dist_VF = new FuzzySet(-2,-2,d0,d1);

//***** Angle *****//
FuzzySet *ang_BN = new FuzzySet(-2000,-2000,-140,-50);
FuzzySet *ang_N = new FuzzySet(-140,-50,-50,-10);
FuzzySet *ang_SN = new FuzzySet(-50,-10,-10,-0.5);
FuzzySet *ang_Z = new FuzzySet(-0.5,0,0,0.5);
FuzzySet *ang_SP = new FuzzySet(0.5,10,10,50);
FuzzySet *ang_P = new FuzzySet(10,50,50,140);
FuzzySet *ang_BP = new FuzzySet(50,140,2000,2000);

//***** Motor 1 *****// (Left)
FuzzySet *left_Z = new FuzzySet(0,0,85,95);
FuzzySet *left_S = new FuzzySet(85, 95, 95, 120);
FuzzySet *left_M = new FuzzySet(95, 120, 120, 150);
FuzzySet *left_B = new FuzzySet(120, 150, 150, 210);
FuzzySet *left_VB = new FuzzySet(150, 210, 235, 235);

//***** Motor 2 *****// (Right)
FuzzySet *right_Z = new FuzzySet(0,0,85,95);
FuzzySet *right_S = new FuzzySet(85, 95, 95, 120);
FuzzySet *right_M = new FuzzySet(95, 120, 120, 150);
FuzzySet *right_B = new FuzzySet(120, 150, 150, 210);
FuzzySet *right_VB = new FuzzySet(150, 210, 235, 235);

void Fuzzy_Docking(){
  //****************************** Start Fuzzy logic for Docking ******************************//
  //********** variables **********//
  //***** variable of X *****//
  FuzzyInput *DOCK_X = new FuzzyInput(1);
  DOCK_X->addFuzzySet(dock_XBN);
  DOCK_X->addFuzzySet(dock_XN);
  DOCK_X->addFuzzySet(dock_XZ);
  DOCK_X->addFuzzySet(dock_XP);
  DOCK_X->addFuzzySet(dock_XBP);
  fuzzy_docking->addFuzzyInput(DOCK_X);

  //**** variable of Z *****//
  FuzzyInput *DOCK_Z = new FuzzyInput(2);
  DOCK_Z->addFuzzySet(dock_ZZ);
  DOCK_Z->addFuzzySet(dock_ZN);
  DOCK_Z->addFuzzySet(dock_ZM);
  DOCK_Z->addFuzzySet(dock_ZF);
  fuzzy_docking->addFuzzyInput(DOCK_Z);

  //***** variable left velocity *****
  FuzzyOutput *leftVelD = new FuzzyOutput(1);
  leftVelD->addFuzzySet(dock_leftBB);
  leftVelD->addFuzzySet(dock_leftBS);
  leftVelD->addFuzzySet(dock_leftZ);
  leftVelD->addFuzzySet(dock_leftFS);
  leftVelD->addFuzzySet(dock_leftFB);
  fuzzy_docking->addFuzzyOutput(leftVelD);

  //***** variable right velocity *****
  FuzzyOutput *rightVelD = new FuzzyOutput(2);
  rightVelD->addFuzzySet(dock_rightBB);
  rightVelD->addFuzzySet(dock_rightBS);
  rightVelD->addFuzzySet(dock_rightZ);
  rightVelD->addFuzzySet(dock_rightFS);
  rightVelD->addFuzzySet(dock_rightFB);
  fuzzy_docking->addFuzzyOutput(rightVelD);

  //***** Rule of Output *****//
  // 1
  FuzzyRuleConsequent *thenLeftZ_RightZD = new FuzzyRuleConsequent();
  thenLeftZ_RightZD->addOutput(dock_leftZ);
  thenLeftZ_RightZD->addOutput(dock_rightZ);
  // 2
  FuzzyRuleConsequent *thenLeftBS_RightBSD = new FuzzyRuleConsequent();
  thenLeftBS_RightBSD->addOutput(dock_leftBS);
  thenLeftBS_RightBSD->addOutput(dock_rightBS);
  // 3
  FuzzyRuleConsequent *thenLeftBB_RightBBD = new FuzzyRuleConsequent();
  thenLeftBB_RightBBD->addOutput(dock_leftBB);
  thenLeftBB_RightBBD->addOutput(dock_rightBB);
  // 4
  FuzzyRuleConsequent *thenLeftFS_RightFSD = new FuzzyRuleConsequent();
  thenLeftFS_RightFSD->addOutput(dock_leftFS);
  thenLeftFS_RightFSD->addOutput(dock_rightFS);
  // 5
  FuzzyRuleConsequent *thenLeftFB_RightFBD = new FuzzyRuleConsequent();
  thenLeftFB_RightFBD->addOutput(dock_leftFB);
  thenLeftFB_RightFBD->addOutput(dock_rightFB);
  // 6
  FuzzyRuleConsequent *thenLeftFS_RightBSD = new FuzzyRuleConsequent();
  thenLeftFS_RightBSD->addOutput(dock_leftFS);
  thenLeftFS_RightBSD->addOutput(dock_rightBS);
  // 7
  FuzzyRuleConsequent *thenLeftBS_RightFSD = new FuzzyRuleConsequent();
  thenLeftBS_RightFSD->addOutput(dock_leftBS);
  thenLeftBS_RightFSD->addOutput(dock_rightFS);
  // 8
  FuzzyRuleConsequent *thenLeftFB_RightBBD = new FuzzyRuleConsequent();
  thenLeftFB_RightBBD->addOutput(dock_leftFB);
  thenLeftFB_RightBBD->addOutput(dock_rightBB);
  // 9
  FuzzyRuleConsequent *thenLeftBB_RightFBD = new FuzzyRuleConsequent();
  thenLeftBB_RightFBD->addOutput(dock_leftBB);
  thenLeftBB_RightFBD->addOutput(dock_rightFB);
  // 10
  FuzzyRuleConsequent *thenLeftFB_RightFSD = new FuzzyRuleConsequent();
  thenLeftFB_RightFSD->addOutput(dock_leftFB);
  thenLeftFB_RightFSD->addOutput(dock_rightFS);
  // 11
  FuzzyRuleConsequent *thenLeftFS_RightFBD = new FuzzyRuleConsequent();
  thenLeftFS_RightFBD->addOutput(dock_leftFS);
  thenLeftFS_RightFBD->addOutput(dock_rightFB);
  // 12
  FuzzyRuleConsequent *thenLeftBB_RightBSD = new FuzzyRuleConsequent();
  thenLeftBB_RightBSD->addOutput(dock_leftBB);
  thenLeftBB_RightBSD->addOutput(dock_rightBS);
  // 13
  FuzzyRuleConsequent *thenLeftBS_RightBBD = new FuzzyRuleConsequent();
  thenLeftBS_RightBBD->addOutput(dock_leftBS);
  thenLeftBS_RightBBD->addOutput(dock_rightBB);
  // 14
  FuzzyRuleConsequent *thenLeftFB_RightZD = new FuzzyRuleConsequent();
  thenLeftFB_RightZD->addOutput(dock_leftFB);
  thenLeftFB_RightZD->addOutput(dock_rightZ);
  // 15
  FuzzyRuleConsequent *thenLeftZ_RightFBD = new FuzzyRuleConsequent();
  thenLeftZ_RightFBD->addOutput(dock_leftZ);
  thenLeftZ_RightFBD->addOutput(dock_rightFB);
  // 16
  FuzzyRuleConsequent *thenLeftFS_RightZD = new FuzzyRuleConsequent();
  thenLeftFS_RightZD->addOutput(dock_leftFS);
  thenLeftFS_RightZD->addOutput(dock_rightZ);
  // 17
  FuzzyRuleConsequent *thenLeftZ_RightFSD = new FuzzyRuleConsequent();
  thenLeftZ_RightFSD->addOutput(dock_leftZ);
  thenLeftZ_RightFSD->addOutput(dock_rightFS);
  // 18
  FuzzyRuleConsequent *thenLeftZ_RightBSD = new FuzzyRuleConsequent();
  thenLeftZ_RightBSD->addOutput(dock_leftZ);
  thenLeftZ_RightBSD->addOutput(dock_rightBS);
  // 19
  FuzzyRuleConsequent *thenLeftBS_RightZD = new FuzzyRuleConsequent();
  thenLeftBS_RightZD->addOutput(dock_leftBS);
  thenLeftBS_RightZD->addOutput(dock_rightZ);
  // 20
  FuzzyRuleConsequent *thenLeftZ_RightBBD = new FuzzyRuleConsequent();
  thenLeftZ_RightBBD->addOutput(dock_leftZ);
  thenLeftZ_RightBBD->addOutput(dock_rightBB);
  // 21
  FuzzyRuleConsequent *thenLeftBB_RightZD = new FuzzyRuleConsequent();
  thenLeftBB_RightZD->addOutput(dock_leftBB);
  thenLeftBB_RightZD->addOutput(dock_rightZ);
  // 22
  FuzzyRuleConsequent *thenLeftFS_RightBBD = new FuzzyRuleConsequent();
  thenLeftFS_RightBBD->addOutput(dock_leftFS);
  thenLeftFS_RightBBD->addOutput(dock_rightBB);
  // 23
  FuzzyRuleConsequent *thenLeftBB_RightFSD = new FuzzyRuleConsequent();
  thenLeftBB_RightFSD->addOutput(dock_leftBB);
  thenLeftBB_RightFSD->addOutput(dock_rightFS);

  // Building Rule 
  // Rule 1
  FuzzyRuleAntecedent *ifZZ_XN = new FuzzyRuleAntecedent();
  ifZZ_XN->joinWithAND(dock_ZZ, dock_XN);
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifZZ_XN, thenLeftBB_RightBBD);
  fuzzy_docking->addFuzzyRule(fuzzyRule1);
  
  // Rule 2
  FuzzyRuleAntecedent *ifZZ_XZ = new FuzzyRuleAntecedent();
  ifZZ_XZ->joinWithAND(dock_ZZ, dock_XZ);
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifZZ_XZ, thenLeftZ_RightZD);
  fuzzy_docking->addFuzzyRule(fuzzyRule2);

  // Rule 3
  FuzzyRuleAntecedent *ifZZ_XP = new FuzzyRuleAntecedent();
  ifZZ_XP->joinWithAND(dock_ZZ, dock_XP);
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifZZ_XP, thenLeftBB_RightBBD);
  fuzzy_docking->addFuzzyRule(fuzzyRule3);

  // Rule 4
  FuzzyRuleAntecedent *ifZN_XN = new FuzzyRuleAntecedent();
  ifZN_XN->joinWithAND(dock_ZN, dock_XN);
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, ifZN_XN, thenLeftFS_RightFBD);
  fuzzy_docking->addFuzzyRule(fuzzyRule4);

  // Rule 5
  FuzzyRuleAntecedent *ifZN_XZ = new FuzzyRuleAntecedent();
  ifZN_XZ->joinWithAND(dock_ZN, dock_XZ);
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, ifZN_XZ, thenLeftFS_RightFSD);
  fuzzy_docking->addFuzzyRule(fuzzyRule5);

  // Rule 6
  FuzzyRuleAntecedent *ifZN_XP = new FuzzyRuleAntecedent();
  ifZN_XP->joinWithAND(dock_ZN, dock_XP);
  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, ifZN_XP, thenLeftFB_RightFSD);
  fuzzy_docking->addFuzzyRule(fuzzyRule6);

  // Rule 7
  FuzzyRuleAntecedent *ifZM_XN = new FuzzyRuleAntecedent();
  ifZM_XN->joinWithAND(dock_ZM, dock_XN);
  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, ifZN_XN, thenLeftFS_RightFBD);
  fuzzy_docking->addFuzzyRule(fuzzyRule7);

  // Rule 8
  FuzzyRuleAntecedent *ifZM_XZ = new FuzzyRuleAntecedent();
  ifZM_XZ->joinWithAND(dock_ZM, dock_XZ);
  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, ifZM_XZ, thenLeftFS_RightFSD);
  fuzzy_docking->addFuzzyRule(fuzzyRule8);

  // Rule 9
  FuzzyRuleAntecedent *ifZM_XP = new FuzzyRuleAntecedent();
  ifZM_XP->joinWithAND(dock_ZM, dock_XP);
  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, ifZM_XP, thenLeftFB_RightFSD);
  fuzzy_docking->addFuzzyRule(fuzzyRule9);

  // Rule 10
  FuzzyRuleAntecedent *ifZF_XN = new FuzzyRuleAntecedent();
  ifZF_XN->joinWithAND(dock_ZF, dock_XN);
  FuzzyRule *fuzzyRule10 = new FuzzyRule(10, ifZF_XN, thenLeftFS_RightFBD);
  fuzzy_docking->addFuzzyRule(fuzzyRule10);

  // Rule 11
  FuzzyRuleAntecedent *ifZF_XZ = new FuzzyRuleAntecedent();
  ifZF_XZ->joinWithAND(dock_ZF, dock_XZ);
  FuzzyRule *fuzzyRule11 = new FuzzyRule(11, ifZF_XZ, thenLeftFB_RightFBD);
  fuzzy_docking->addFuzzyRule(fuzzyRule11);

  // Rule 12
  FuzzyRuleAntecedent *ifZF_XP = new FuzzyRuleAntecedent();
  ifZF_XP->joinWithAND(dock_ZF, dock_XP);
  FuzzyRule *fuzzyRule12 = new FuzzyRule(12, ifZF_XP, thenLeftFB_RightFSD);
  fuzzy_docking->addFuzzyRule(fuzzyRule12);

  // Rule 13
  FuzzyRuleAntecedent *ifZZ_XBN = new FuzzyRuleAntecedent();
  ifZZ_XBN->joinWithAND(dock_ZZ, dock_XBN);
  FuzzyRule *fuzzyRule13 = new FuzzyRule(13, ifZZ_XBN, thenLeftBS_RightBBD);
  fuzzy_docking->addFuzzyRule(fuzzyRule13);

  // Rule 14
  FuzzyRuleAntecedent *ifZZ_XBP = new FuzzyRuleAntecedent();
  ifZZ_XBP->joinWithAND(dock_ZZ, dock_XBP);
  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, ifZZ_XBP, thenLeftBB_RightBSD);
  fuzzy_docking->addFuzzyRule(fuzzyRule14);

  // Rule 15
  FuzzyRuleAntecedent *ifZN_XBN = new FuzzyRuleAntecedent();
  ifZN_XBN->joinWithAND(dock_ZN, dock_XBN);
  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, ifZN_XBN, thenLeftBS_RightBBD);
  fuzzy_docking->addFuzzyRule(fuzzyRule15);

  // Rule 16
  FuzzyRuleAntecedent *ifZN_XBP = new FuzzyRuleAntecedent();
  ifZN_XBP->joinWithAND(dock_ZN, dock_XBP);
  FuzzyRule *fuzzyRule16 = new FuzzyRule(16, ifZN_XBP, thenLeftBB_RightBSD);
  fuzzy_docking->addFuzzyRule(fuzzyRule16);

  // Rule 17
  FuzzyRuleAntecedent *ifZM_XBN = new FuzzyRuleAntecedent();
  ifZM_XBN->joinWithAND(dock_ZM, dock_XBN);
  FuzzyRule *fuzzyRule17 = new FuzzyRule(17, ifZM_XBN, thenLeftFS_RightFBD);
  fuzzy_docking->addFuzzyRule(fuzzyRule17);

  // Rule 18
  FuzzyRuleAntecedent *ifZM_XBP = new FuzzyRuleAntecedent();
  ifZM_XBP->joinWithAND(dock_ZM, dock_XBP);
  FuzzyRule *fuzzyRule18 = new FuzzyRule(18, ifZM_XBP, thenLeftFB_RightFSD);
  fuzzy_docking->addFuzzyRule(fuzzyRule18);

  // Rule 19
  FuzzyRuleAntecedent *ifZF_XBN = new FuzzyRuleAntecedent();
  ifZF_XBN->joinWithAND(dock_ZF, dock_XBN);
  FuzzyRule *fuzzyRule19 = new FuzzyRule(19, ifZF_XBN, thenLeftZ_RightFSD);
  fuzzy_docking->addFuzzyRule(fuzzyRule19);

  // Rule 20
  FuzzyRuleAntecedent *ifZF_XBP = new FuzzyRuleAntecedent();
  ifZF_XBP->joinWithAND(dock_ZF, dock_XBP);
  FuzzyRule *fuzzyRule20 = new FuzzyRule(20, ifZF_XBP, thenLeftFS_RightZD);
  fuzzy_docking->addFuzzyRule(fuzzyRule20);

}
void Fuzzy_Obstacle(){
  //****************************** Start Fuzzy logic for Obstacle ******************************//
  //********** variables **********//
  //***** variable of left sensor *****//
  FuzzyInput *DLO = new FuzzyInput(1);
  DLO->addFuzzySet(near1);
  DLO->addFuzzySet(mid1);
  DLO->addFuzzySet(far1);
  fuzzy_obstacle->addFuzzyInput(DLO);

  //**** variable of center sensor *****//
  FuzzyInput *DCO = new FuzzyInput(2);
  DCO->addFuzzySet(near2);
  DCO->addFuzzySet(mid2);
  DCO->addFuzzySet(far2);
  fuzzy_obstacle->addFuzzyInput(DCO);

  //**** variable of right sensor *****//
  FuzzyInput *DRO = new FuzzyInput(3);
  DRO->addFuzzySet(near3);
  DRO->addFuzzySet(mid3);
  DRO->addFuzzySet(far3);
  fuzzy_obstacle->addFuzzyInput(DRO);

  //***** variable left velocity *****
  FuzzyOutput *leftVelO = new FuzzyOutput(1);
  leftVelO->addFuzzySet(leftBB);
  leftVelO->addFuzzySet(leftBS);
  leftVelO->addFuzzySet(leftZ);
  leftVelO->addFuzzySet(leftFS);
  leftVelO->addFuzzySet(leftFB);
  fuzzy_obstacle->addFuzzyOutput(leftVelO);

  //***** variable right velocity *****
  FuzzyOutput *rightVelO = new FuzzyOutput(2);
  rightVelO->addFuzzySet(rightBB);
  rightVelO->addFuzzySet(rightBS);
  rightVelO->addFuzzySet(rightZ);
  rightVelO->addFuzzySet(rightFS);
  rightVelO->addFuzzySet(rightFB);
  fuzzy_obstacle->addFuzzyOutput(rightVelO); 

  //***** Building Fuzzy rules *****//
  //***** Rule of Left & Center only (Input) *****//
  // 1
  FuzzyRuleAntecedent *DLNear_DCNearO = new FuzzyRuleAntecedent();
  DLNear_DCNearO->joinWithAND(near1, near2);
  // 2
  FuzzyRuleAntecedent *DLFar_DCFarO = new FuzzyRuleAntecedent();
  DLFar_DCFarO->joinWithAND(far1, far2);
  // 3 
  FuzzyRuleAntecedent *DLMid_DCMidO = new FuzzyRuleAntecedent();
  DLMid_DCMidO->joinWithAND(mid1, mid2);
  // 4
  FuzzyRuleAntecedent *DLNear_DCMidO = new FuzzyRuleAntecedent();
  DLNear_DCMidO->joinWithAND(near1, mid2);
  // 5
  FuzzyRuleAntecedent *DLMid_DCNearO = new FuzzyRuleAntecedent();
  DLMid_DCNearO->joinWithAND(mid1, near2);
  // 6
  FuzzyRuleAntecedent *DLFar_DCNearO = new FuzzyRuleAntecedent();
  DLFar_DCNearO->joinWithAND(far1, near2);
  // 7
  FuzzyRuleAntecedent *DLNear_DCFarO = new FuzzyRuleAntecedent();
  DLNear_DCFarO->joinWithAND(near1, far2);
  // 8
  FuzzyRuleAntecedent *DLMid_DCFarO = new FuzzyRuleAntecedent();
  DLMid_DCFarO->joinWithAND(mid1, far2);
  // 9
  FuzzyRuleAntecedent *DLFar_DCMidO = new FuzzyRuleAntecedent();
  DLFar_DCMidO->joinWithAND(far1, mid2);

  //***** Rule of Output *****//
  // 1
  FuzzyRuleConsequent *thenLeftZ_RightZO = new FuzzyRuleConsequent();
  thenLeftZ_RightZO->addOutput(leftZ);
  thenLeftZ_RightZO->addOutput(rightZ);
  // 2
  FuzzyRuleConsequent *thenLeftBS_RightBSO = new FuzzyRuleConsequent();
  thenLeftBS_RightBSO->addOutput(leftBS);
  thenLeftBS_RightBSO->addOutput(rightBS);
  // 3
  FuzzyRuleConsequent *thenLeftBB_RightBBO = new FuzzyRuleConsequent();
  thenLeftBB_RightBBO->addOutput(leftBB);
  thenLeftBB_RightBBO->addOutput(rightBB);
  // 4
  FuzzyRuleConsequent *thenLeftFS_RightFSO = new FuzzyRuleConsequent();
  thenLeftFS_RightFSO->addOutput(leftFS);
  thenLeftFS_RightFSO->addOutput(rightFS);
  // 5
  FuzzyRuleConsequent *thenLeftFB_RightFBO = new FuzzyRuleConsequent();
  thenLeftFB_RightFBO->addOutput(leftFB);
  thenLeftFB_RightFBO->addOutput(rightFB);
  // 6
  FuzzyRuleConsequent *thenLeftFS_RightBSO = new FuzzyRuleConsequent();
  thenLeftFS_RightBSO->addOutput(leftFS);
  thenLeftFS_RightBSO->addOutput(rightBS);
  // 7
  FuzzyRuleConsequent *thenLeftBS_RightFSO = new FuzzyRuleConsequent();
  thenLeftBS_RightFSO->addOutput(leftBS);
  thenLeftBS_RightFSO->addOutput(rightFS);
  // 8
  FuzzyRuleConsequent *thenLeftFB_RightBBO = new FuzzyRuleConsequent();
  thenLeftFB_RightBBO->addOutput(leftFB);
  thenLeftFB_RightBBO->addOutput(rightBB);
  // 9
  FuzzyRuleConsequent *thenLeftBB_RightFBO = new FuzzyRuleConsequent();
  thenLeftBB_RightFBO->addOutput(leftBB);
  thenLeftBB_RightFBO->addOutput(rightFB);
  // 10
  FuzzyRuleConsequent *thenLeftFB_RightFSO = new FuzzyRuleConsequent();
  thenLeftFB_RightFSO->addOutput(leftFB);
  thenLeftFB_RightFSO->addOutput(rightFS);
  // 11
  FuzzyRuleConsequent *thenLeftFS_RightFBO = new FuzzyRuleConsequent();
  thenLeftFS_RightFBO->addOutput(leftFS);
  thenLeftFS_RightFBO->addOutput(rightFB);
  // 12
  FuzzyRuleConsequent *thenLeftBB_RightBSO = new FuzzyRuleConsequent();
  thenLeftBB_RightBSO->addOutput(leftBB);
  thenLeftBB_RightBSO->addOutput(rightBS);
  // 13
  FuzzyRuleConsequent *thenLeftBS_RightBBO = new FuzzyRuleConsequent();
  thenLeftBS_RightBBO->addOutput(leftBS);
  thenLeftBS_RightBBO->addOutput(rightBB);
  // 14
  FuzzyRuleConsequent *thenLeftFB_RightZO = new FuzzyRuleConsequent();
  thenLeftFB_RightZO->addOutput(leftFB);
  thenLeftFB_RightZO->addOutput(rightZ);
  // 15
  FuzzyRuleConsequent *thenLeftZ_RightFBO = new FuzzyRuleConsequent();
  thenLeftZ_RightFBO->addOutput(leftZ);
  thenLeftZ_RightFBO->addOutput(rightFB);
  // 16
  FuzzyRuleConsequent *thenLeftFS_RightZO = new FuzzyRuleConsequent();
  thenLeftFS_RightZO->addOutput(leftFS);
  thenLeftFS_RightZO->addOutput(rightZ);
  // 17
  FuzzyRuleConsequent *thenLeftZ_RightFSO = new FuzzyRuleConsequent();
  thenLeftZ_RightFSO->addOutput(leftZ);
  thenLeftZ_RightFSO->addOutput(rightFS);
  // 18
  FuzzyRuleConsequent *thenLeftZ_RightBSO = new FuzzyRuleConsequent();
  thenLeftZ_RightBSO->addOutput(leftZ);
  thenLeftZ_RightBSO->addOutput(rightBS);
  // 19
  FuzzyRuleConsequent *thenLeftBS_RightZO = new FuzzyRuleConsequent();
  thenLeftBS_RightZO->addOutput(leftBS);
  thenLeftBS_RightZO->addOutput(rightZ);
  // 20
  FuzzyRuleConsequent *thenLeftZ_RightBBO = new FuzzyRuleConsequent();
  thenLeftZ_RightBBO->addOutput(leftZ);
  thenLeftZ_RightBBO->addOutput(rightBB);
  // 21
  FuzzyRuleConsequent *thenLeftBB_RightZO = new FuzzyRuleConsequent();
  thenLeftBB_RightZO->addOutput(leftBB);
  thenLeftBB_RightZO->addOutput(rightZ);
  // 22
  FuzzyRuleConsequent *thenLeftFS_RightBBO = new FuzzyRuleConsequent();
  thenLeftFS_RightBBO->addOutput(leftFS);
  thenLeftFS_RightBBO->addOutput(rightBB);
  // 23
  FuzzyRuleConsequent *thenLeftBB_RightFSO = new FuzzyRuleConsequent();
  thenLeftBB_RightFSO->addOutput(leftBB);
  thenLeftBB_RightFSO->addOutput(rightFS);

  // Building Rule 
  // Rule 1
  FuzzyRuleAntecedent *ifDLNear_DCNear_DRNearO = new FuzzyRuleAntecedent();
  ifDLNear_DCNear_DRNearO->joinWithAND(DLNear_DCNearO, near3);
  FuzzyRule *fuzzyRuleO1 = new FuzzyRule(1, ifDLNear_DCNear_DRNearO, thenLeftZ_RightBSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO1);

  // Rule 2
  FuzzyRuleAntecedent *ifDLNear_DCNear_DRMidO = new FuzzyRuleAntecedent();
  ifDLNear_DCNear_DRMidO->joinWithAND(DLNear_DCNearO, mid3);
  FuzzyRule *fuzzyRuleO2 = new FuzzyRule(2, ifDLNear_DCNear_DRMidO, thenLeftZ_RightBSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO2);

  // Rule 3
  FuzzyRuleAntecedent *ifDLNear_DCNear_DRFarO = new FuzzyRuleAntecedent();
  ifDLNear_DCNear_DRFarO->joinWithAND(DLNear_DCNearO, far3);
  FuzzyRule *fuzzyRuleO3 = new FuzzyRule(3, ifDLNear_DCNear_DRFarO, thenLeftZ_RightBBO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO3);

  // Rule 4
  FuzzyRuleAntecedent *ifDLNear_DCMid_DRNearO = new FuzzyRuleAntecedent();
  ifDLNear_DCMid_DRNearO->joinWithAND(DLNear_DCMidO, near3);
  FuzzyRule *fuzzyRuleO4 = new FuzzyRule(4, ifDLNear_DCMid_DRNearO, thenLeftZ_RightFSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO4);

  // Rule 5
  FuzzyRuleAntecedent *ifDLNear_DCMid_DRMidO = new FuzzyRuleAntecedent();
  ifDLNear_DCMid_DRMidO->joinWithAND(DLNear_DCMidO, mid3);
  FuzzyRule *fuzzyRuleO5 = new FuzzyRule(5, ifDLNear_DCMid_DRMidO, thenLeftZ_RightBSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO5);

  // Rule 6
  FuzzyRuleAntecedent *ifDLNear_DCMid_DRFarO = new FuzzyRuleAntecedent();
  ifDLNear_DCMid_DRFarO->joinWithAND(DLNear_DCMidO, far3);
  FuzzyRule *fuzzyRuleO6 = new FuzzyRule(6, ifDLNear_DCMid_DRFarO, thenLeftZ_RightBBO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO6);

  // Rule 7
  FuzzyRuleAntecedent *ifDLNear_DCFar_DRNearO = new FuzzyRuleAntecedent();
  ifDLNear_DCFar_DRNearO->joinWithAND(DLNear_DCFarO, near3);
  FuzzyRule *fuzzyRuleO7 = new FuzzyRule(7, ifDLNear_DCFar_DRNearO, thenLeftBS_RightZO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO7);

  // Rule 8
  FuzzyRuleAntecedent *ifDLNear_DCFar_DRMidO = new FuzzyRuleAntecedent();
  ifDLNear_DCFar_DRMidO->joinWithAND(DLNear_DCFarO, mid3);
  FuzzyRule *fuzzyRuleO8 = new FuzzyRule(8, ifDLNear_DCFar_DRMidO, thenLeftFS_RightBBO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO8);

  // Rule 9
  FuzzyRuleAntecedent *ifDLNear_DCFar_DRFarO = new FuzzyRuleAntecedent();
  ifDLNear_DCFar_DRFarO->joinWithAND(DLNear_DCFarO, far3);
  FuzzyRule *fuzzyRuleO9 = new FuzzyRule(9, ifDLNear_DCFar_DRFarO, thenLeftFS_RightBBO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO9);

  // Rule 10
  FuzzyRuleAntecedent *ifDLMid_DCNear_DRNearO = new FuzzyRuleAntecedent();
  ifDLMid_DCNear_DRNearO->joinWithAND(DLMid_DCNearO, near3);
  FuzzyRule *fuzzyRuleO10 = new FuzzyRule(10, ifDLMid_DCNear_DRNearO, thenLeftBS_RightZO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO10);

  // Rule 11
  FuzzyRuleAntecedent *ifDLMid_DCNear_DRMidO = new FuzzyRuleAntecedent();
  ifDLMid_DCNear_DRMidO->joinWithAND(DLMid_DCNearO, mid3);
  FuzzyRule *fuzzyRuleO11 = new FuzzyRule(11, ifDLMid_DCNear_DRMidO, thenLeftBS_RightZO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO11);

  // Rule 12
  FuzzyRuleAntecedent *ifDLMid_DCNear_DRFarO = new FuzzyRuleAntecedent();
  ifDLMid_DCNear_DRFarO->joinWithAND(DLMid_DCNearO, far3);
  FuzzyRule *fuzzyRuleO12 = new FuzzyRule(12, ifDLMid_DCNear_DRFarO, thenLeftZ_RightBBO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO12);

  // Rule 13
  FuzzyRuleAntecedent *ifDLMid_DCMid_DRNearO = new FuzzyRuleAntecedent();
  ifDLMid_DCMid_DRNearO->joinWithAND(DLMid_DCMidO, near3);
  FuzzyRule *fuzzyRuleO13 = new FuzzyRule(13, ifDLMid_DCMid_DRNearO, thenLeftBB_RightFSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO13);

  // Rule 14
  FuzzyRuleAntecedent *ifDLMid_DCMid_DRMidO = new FuzzyRuleAntecedent();
  ifDLMid_DCMid_DRMidO->joinWithAND(DLMid_DCMidO, mid3);
  FuzzyRule *fuzzyRuleO14 = new FuzzyRule(14, ifDLMid_DCMid_DRMidO, thenLeftFS_RightFBO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO14);

  // Rule 15
  FuzzyRuleAntecedent *ifDLMid_DCMid_DRFarO = new FuzzyRuleAntecedent();
  ifDLMid_DCMid_DRFarO->joinWithAND(DLMid_DCMidO, far3);
  FuzzyRule *fuzzyRuleO15 = new FuzzyRule(15, ifDLMid_DCMid_DRFarO, thenLeftFB_RightFSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO15);

  // Rule 16
  FuzzyRuleAntecedent *ifDLMid_DCFar_DRNearO = new FuzzyRuleAntecedent();
  ifDLMid_DCFar_DRNearO->joinWithAND(DLMid_DCFarO, near3);
  FuzzyRule *fuzzyRuleO16 = new FuzzyRule(16, ifDLMid_DCFar_DRNearO, thenLeftBB_RightFSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO16);

  // Rule 17
  FuzzyRuleAntecedent *ifDLMid_DCFar_DRMidO = new FuzzyRuleAntecedent();
  ifDLMid_DCFar_DRMidO->joinWithAND(DLMid_DCFarO, mid3);
  FuzzyRule *fuzzyRuleO17 = new FuzzyRule(17, ifDLMid_DCFar_DRMidO, thenLeftFS_RightFBO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO17);

  // Rule 18
  FuzzyRuleAntecedent *ifDLMid_DCFar_DRFarO = new FuzzyRuleAntecedent();
  ifDLMid_DCFar_DRFarO->joinWithAND(DLMid_DCFarO, far3);
  FuzzyRule *fuzzyRuleO18 = new FuzzyRule(18, ifDLMid_DCFar_DRFarO, thenLeftFB_RightFSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO18);

  // Rule 19
  FuzzyRuleAntecedent *ifDLFar_DCNear_DRNearO = new FuzzyRuleAntecedent();
  ifDLFar_DCNear_DRNearO->joinWithAND(DLFar_DCNearO, near3);
  FuzzyRule *fuzzyRuleO19 = new FuzzyRule(19, ifDLFar_DCNear_DRNearO, thenLeftBB_RightZO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO19);

  // Rule 20
  FuzzyRuleAntecedent *ifDLFar_DCNear_DRMidO = new FuzzyRuleAntecedent();
  ifDLFar_DCNear_DRMidO->joinWithAND(DLFar_DCNearO, mid3);
  FuzzyRule *fuzzyRuleO20 = new FuzzyRule(20, ifDLFar_DCNear_DRMidO, thenLeftBB_RightZO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO20);

  // Rule 21
  FuzzyRuleAntecedent *ifDLFar_DCNear_DRFarO = new FuzzyRuleAntecedent();
  ifDLFar_DCNear_DRFarO->joinWithAND(DLFar_DCNearO, far3);
  FuzzyRule *fuzzyRuleO21 = new FuzzyRule(21, ifDLFar_DCNear_DRFarO, thenLeftBS_RightFSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO21);

  // Rule 22
  FuzzyRuleAntecedent *ifDLFar_DCMid_DRNearO = new FuzzyRuleAntecedent();
  ifDLFar_DCMid_DRNearO->joinWithAND(DLFar_DCMidO, near3);
  FuzzyRule *fuzzyRuleO22 = new FuzzyRule(22, ifDLFar_DCMid_DRNearO, thenLeftBB_RightZO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO22);

  // Rule 23
  FuzzyRuleAntecedent *ifDLFar_DCMid_DRMidO = new FuzzyRuleAntecedent();
  ifDLFar_DCMid_DRMidO->joinWithAND(DLFar_DCMidO, mid3);
  FuzzyRule *fuzzyRuleO23 = new FuzzyRule(23, ifDLFar_DCMid_DRMidO, thenLeftFS_RightFBO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO23);

  // Rule 24
  FuzzyRuleAntecedent *ifDLFar_DCMid_DRFarO = new FuzzyRuleAntecedent();
  ifDLFar_DCMid_DRFarO->joinWithAND(DLFar_DCMidO, far3);
  FuzzyRule *fuzzyRuleO24 = new FuzzyRule(24, ifDLFar_DCMid_DRFarO, thenLeftFS_RightFBO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO24);

  // Rule 25
  FuzzyRuleAntecedent *ifDLFar_DCFar_DRNearO = new FuzzyRuleAntecedent();
  ifDLFar_DCFar_DRNearO->joinWithAND(DLFar_DCFarO, near3);
  FuzzyRule *fuzzyRuleO25 = new FuzzyRule(25, ifDLFar_DCFar_DRNearO, thenLeftBB_RightFSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO25);

  // Rule 26
  FuzzyRuleAntecedent *ifDLFar_DCfar_DRmidO = new FuzzyRuleAntecedent();
  ifDLFar_DCfar_DRmidO->joinWithAND(DLFar_DCFarO, mid3);
  FuzzyRule *fuzzyRuleO26 = new FuzzyRule(26, ifDLFar_DCfar_DRmidO, thenLeftBB_RightFSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO26);

  // Rule 27
  FuzzyRuleAntecedent *ifDLFar_DCFar_DRFarO = new FuzzyRuleAntecedent();
  ifDLFar_DCFar_DRFarO->joinWithAND(DLFar_DCFarO, far3);
  FuzzyRule *fuzzyRuleO27 = new FuzzyRule(27, ifDLFar_DCFar_DRFarO, thenLeftFS_RightFSO);
  fuzzy_obstacle->addFuzzyRule(fuzzyRuleO27);
//****************************** Start Fuzzy logic for Obstacle ******************************//
}

void Fuzzy_Tracking(){
  //****************************** Start Fuzzy logic for Tracking ******************************//
  //********** variables **********//
  FuzzyInput *Dist = new FuzzyInput(1);
  Dist->addFuzzySet(dist_Z);
  Dist->addFuzzySet(dist_VN);
  Dist->addFuzzySet(dist_N);
  Dist->addFuzzySet(dist_M);
  Dist->addFuzzySet(dist_SF);
  Dist->addFuzzySet(dist_F);
  Dist->addFuzzySet(dist_VF);
  fuzzy_tracking->addFuzzyInput(Dist);

  FuzzyInput *Angle = new FuzzyInput(2);
  Angle->addFuzzySet(ang_BN);
  Angle->addFuzzySet(ang_N);
  Angle->addFuzzySet(ang_SN);
  Angle->addFuzzySet(ang_Z);
  Angle->addFuzzySet(ang_SP);
  Angle->addFuzzySet(ang_P);
  Angle->addFuzzySet(ang_BP);
  fuzzy_tracking->addFuzzyInput(Angle);

  //***** variable left velocity *****
  FuzzyOutput *VL = new FuzzyOutput(1);
  VL->addFuzzySet(left_Z);
  VL->addFuzzySet(left_S);
  VL->addFuzzySet(left_M);
  VL->addFuzzySet(left_B);
  VL->addFuzzySet(left_VB);
  fuzzy_tracking->addFuzzyOutput(VL); 

  //***** variable right velocity *****
  FuzzyOutput *VR = new FuzzyOutput(2);
  VR->addFuzzySet(right_Z);
  VR->addFuzzySet(right_S);
  VR->addFuzzySet(right_M);
  VR->addFuzzySet(right_B);
  VR->addFuzzySet(right_VB);
  fuzzy_tracking->addFuzzyOutput(VR); 

  //***** Building Fuzzy rules *****//
  //***** Rule of Output *****//
  // 1
  FuzzyRuleConsequent *thenLeftZ_RightZ = new FuzzyRuleConsequent();
  thenLeftZ_RightZ->addOutput(left_Z);
  thenLeftZ_RightZ->addOutput(right_Z);
  // 2
  FuzzyRuleConsequent *thenLeftB_RightB = new FuzzyRuleConsequent();
  thenLeftB_RightB->addOutput(left_B);
  thenLeftB_RightB->addOutput(right_B);
  // 3
  FuzzyRuleConsequent *thenLeftS_RightS = new FuzzyRuleConsequent();
  thenLeftS_RightS->addOutput(left_S);
  thenLeftS_RightS->addOutput(right_S);
  // 4
  FuzzyRuleConsequent *thenLeftM_RightM = new FuzzyRuleConsequent();
  thenLeftM_RightM->addOutput(left_M);
  thenLeftM_RightM->addOutput(right_M);
  // 5
  FuzzyRuleConsequent *thenLeftVB_RightVB = new FuzzyRuleConsequent();
  thenLeftVB_RightVB->addOutput(left_VB);
  thenLeftVB_RightVB->addOutput(right_VB);
  // 6
  FuzzyRuleConsequent *thenLeftZ_RightM = new FuzzyRuleConsequent();
  thenLeftZ_RightM->addOutput(left_Z);
  thenLeftZ_RightM->addOutput(right_M);
  // 7
  FuzzyRuleConsequent *thenLeftM_RightZ = new FuzzyRuleConsequent();
  thenLeftM_RightZ->addOutput(left_M);
  thenLeftM_RightZ->addOutput(right_Z);
  // 8
  FuzzyRuleConsequent *thenLeftZ_RightS = new FuzzyRuleConsequent();
  thenLeftZ_RightS->addOutput(left_Z);
  thenLeftZ_RightS->addOutput(right_S);
  // 9
  FuzzyRuleConsequent *thenLeftS_RightZ = new FuzzyRuleConsequent();
  thenLeftS_RightZ->addOutput(left_S);
  thenLeftS_RightZ->addOutput(right_Z);
  // 10
  FuzzyRuleConsequent *thenLeftS_RightB = new FuzzyRuleConsequent();
  thenLeftS_RightB->addOutput(left_S);
  thenLeftS_RightB->addOutput(right_B);
  // 11
  FuzzyRuleConsequent *thenLeftB_RightS = new FuzzyRuleConsequent();
  thenLeftB_RightS->addOutput(left_B);
  thenLeftB_RightS->addOutput(right_S);
  // 12
  FuzzyRuleConsequent *thenLeftS_RightVB = new FuzzyRuleConsequent();
  thenLeftS_RightVB->addOutput(left_S);
  thenLeftS_RightVB->addOutput(right_VB);
  // 13
  FuzzyRuleConsequent *thenLeftVB_RightS = new FuzzyRuleConsequent();
  thenLeftVB_RightS->addOutput(left_VB);
  thenLeftVB_RightS->addOutput(right_S);
  // 14
  FuzzyRuleConsequent *thenLeftB_RightVB = new FuzzyRuleConsequent();
  thenLeftB_RightVB->addOutput(left_B);
  thenLeftB_RightVB->addOutput(right_VB);
  // 15
  FuzzyRuleConsequent *thenLeftVB_RightB = new FuzzyRuleConsequent();
  thenLeftVB_RightB->addOutput(left_VB);
  thenLeftVB_RightB->addOutput(right_B);
  // 16
  FuzzyRuleConsequent *thenLeftB_RightM = new FuzzyRuleConsequent();
  thenLeftB_RightM->addOutput(left_B);
  thenLeftB_RightM->addOutput(right_M);
  // 17
  FuzzyRuleConsequent *thenLeftM_RightB = new FuzzyRuleConsequent();
  thenLeftM_RightB->addOutput(left_M);
  thenLeftM_RightB->addOutput(right_B);

  // Building Rule 
  // Rule 1
  FuzzyRuleAntecedent *ifDistZ_AngBN = new FuzzyRuleAntecedent();
  ifDistZ_AngBN->joinWithAND(dist_Z, ang_BN);
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifDistZ_AngBN, thenLeftZ_RightM);
  fuzzy_tracking->addFuzzyRule(fuzzyRule1);
  // Rule 2
  FuzzyRuleAntecedent *ifDistZ_AngN = new FuzzyRuleAntecedent();
  ifDistZ_AngN->joinWithAND(dist_Z, ang_N);
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifDistZ_AngN, thenLeftZ_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule2);
  // Rule 3
  FuzzyRuleAntecedent *ifDistZ_AngSN = new FuzzyRuleAntecedent();
  ifDistZ_AngSN->joinWithAND(dist_Z, ang_SN);
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifDistZ_AngSN, thenLeftZ_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule3);
  // Rule 4
  FuzzyRuleAntecedent *ifDistZ_AngZ = new FuzzyRuleAntecedent();
  ifDistZ_AngZ->joinWithAND(dist_Z, ang_Z);
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, ifDistZ_AngZ, thenLeftZ_RightZ);
  fuzzy_tracking->addFuzzyRule(fuzzyRule4);
  // Rule 5
  FuzzyRuleAntecedent *ifDistZ_AngSP = new FuzzyRuleAntecedent();
  ifDistZ_AngSP->joinWithAND(dist_Z, ang_SP);
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, ifDistZ_AngSP, thenLeftS_RightZ);
  fuzzy_tracking->addFuzzyRule(fuzzyRule5);
  // Rule 6
  FuzzyRuleAntecedent *ifDistZ_AngP = new FuzzyRuleAntecedent();
  ifDistZ_AngP->joinWithAND(dist_Z, ang_P);
  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, ifDistZ_AngP, thenLeftS_RightZ);
  fuzzy_tracking->addFuzzyRule(fuzzyRule6);
  // Rule 7
  FuzzyRuleAntecedent *ifDistZ_AngBP = new FuzzyRuleAntecedent();
  ifDistZ_AngBP->joinWithAND(dist_Z, ang_BP);
  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, ifDistZ_AngBP, thenLeftM_RightZ);
  fuzzy_tracking->addFuzzyRule(fuzzyRule7);
  // Rule 8
  FuzzyRuleAntecedent *ifDistVN_AngBN = new FuzzyRuleAntecedent();
  ifDistVN_AngBN->joinWithAND(dist_VN, ang_BN);
  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, ifDistVN_AngBN, thenLeftS_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule8);
  // Rule 9
  FuzzyRuleAntecedent *ifDistVN_AngN = new FuzzyRuleAntecedent();
  ifDistVN_AngN->joinWithAND(dist_VN, ang_N);
  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, ifDistVN_AngN, thenLeftS_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule9);
  // Rule 10
  FuzzyRuleAntecedent *ifDistVN_AngSN = new FuzzyRuleAntecedent();
  ifDistVN_AngSN->joinWithAND(dist_VN, ang_SN);
  FuzzyRule *fuzzyRule10 = new FuzzyRule(10, ifDistVN_AngSN, thenLeftZ_RightM);
  fuzzy_tracking->addFuzzyRule(fuzzyRule10);
  // Rule 11
  FuzzyRuleAntecedent *ifDistVN_AngZ = new FuzzyRuleAntecedent();
  ifDistVN_AngZ->joinWithAND(dist_VN, ang_Z);
  FuzzyRule *fuzzyRule11 = new FuzzyRule(11, ifDistVN_AngZ, thenLeftS_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule11);
  // Rule 12
  FuzzyRuleAntecedent *ifDistVN_AngSP = new FuzzyRuleAntecedent();
  ifDistVN_AngSP->joinWithAND(dist_VN, ang_SP);
  FuzzyRule *fuzzyRule12 = new FuzzyRule(12, ifDistVN_AngSP, thenLeftM_RightZ);
  fuzzy_tracking->addFuzzyRule(fuzzyRule12);
  // Rule 13
  FuzzyRuleAntecedent *ifDistVN_AngP = new FuzzyRuleAntecedent();
  ifDistVN_AngP->joinWithAND(dist_VN, ang_P);
  FuzzyRule *fuzzyRule13 = new FuzzyRule(13, ifDistVN_AngP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule13);
  // Rule 14
  FuzzyRuleAntecedent *ifDistVN_AngBP = new FuzzyRuleAntecedent();
  ifDistVN_AngBP->joinWithAND(dist_VN, ang_BP);
  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, ifDistVN_AngBP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule14);
  // Rule 15
  FuzzyRuleAntecedent *ifDistN_AngBN = new FuzzyRuleAntecedent();
  ifDistN_AngBN->joinWithAND(dist_N, ang_BN);
  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, ifDistN_AngBN, thenLeftS_RightVB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule15);
  // Rule 16
  FuzzyRuleAntecedent *ifDistN_AngN = new FuzzyRuleAntecedent();
  ifDistN_AngN->joinWithAND(dist_N, ang_N);
  FuzzyRule *fuzzyRule16 = new FuzzyRule(16, ifDistN_AngN, thenLeftS_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule16);
  // Rule 17
  FuzzyRuleAntecedent *ifDistN_AngSN = new FuzzyRuleAntecedent();
  ifDistN_AngSN->joinWithAND(dist_N, ang_SN);
  FuzzyRule *fuzzyRule17 = new FuzzyRule(17, ifDistN_AngSN, thenLeftS_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule17);
  // Rule 18
  FuzzyRuleAntecedent *ifDistN_AngZ = new FuzzyRuleAntecedent();
  ifDistN_AngZ->joinWithAND(dist_N, ang_Z);
  FuzzyRule *fuzzyRule18 = new FuzzyRule(18, ifDistN_AngZ, thenLeftS_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule18);
  // Rule 19
  FuzzyRuleAntecedent *ifDistN_AngSP = new FuzzyRuleAntecedent();
  ifDistN_AngSP->joinWithAND(dist_N, ang_SP);
  FuzzyRule *fuzzyRule19 = new FuzzyRule(19, ifDistN_AngSP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule19);
  // Rule 20
  FuzzyRuleAntecedent *ifDistN_AngP = new FuzzyRuleAntecedent();
  ifDistN_AngP->joinWithAND(dist_N, ang_P);
  FuzzyRule *fuzzyRule20 = new FuzzyRule(20, ifDistN_AngP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule20);
  // Rule 21
  FuzzyRuleAntecedent *ifDistN_AngBP = new FuzzyRuleAntecedent();
  ifDistN_AngBP->joinWithAND(dist_N, ang_BP);
  FuzzyRule *fuzzyRule21 = new FuzzyRule(21, ifDistN_AngBP, thenLeftVB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule21);
  // Rule 22
  FuzzyRuleAntecedent *ifDistM_AngBN = new FuzzyRuleAntecedent();
  ifDistM_AngBN->joinWithAND(dist_M, ang_BN);
  FuzzyRule *fuzzyRule22 = new FuzzyRule(22, ifDistM_AngBN, thenLeftS_RightVB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule22);
  // Rule 23
  FuzzyRuleAntecedent *ifDistM_AngN = new FuzzyRuleAntecedent();
  ifDistM_AngN->joinWithAND(dist_M, ang_N);
  FuzzyRule *fuzzyRule23 = new FuzzyRule(23, ifDistM_AngN, thenLeftS_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule23);
  // Rule 24
  FuzzyRuleAntecedent *ifDistM_AngSN = new FuzzyRuleAntecedent();
  ifDistM_AngSN->joinWithAND(dist_M, ang_SN);
  FuzzyRule *fuzzyRule24 = new FuzzyRule(24, ifDistM_AngSN, thenLeftS_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule24);
  // Rule 25
  FuzzyRuleAntecedent *ifDistM_AngZ = new FuzzyRuleAntecedent();
  ifDistM_AngZ->joinWithAND(dist_M, ang_Z);
  FuzzyRule *fuzzyRule25 = new FuzzyRule(25, ifDistM_AngZ, thenLeftM_RightM);
  fuzzy_tracking->addFuzzyRule(fuzzyRule25);
  // Rule 26
  FuzzyRuleAntecedent *ifDistM_AngSP = new FuzzyRuleAntecedent();
  ifDistM_AngSP->joinWithAND(dist_M, ang_SP);
  FuzzyRule *fuzzyRule26 = new FuzzyRule(26, ifDistM_AngSP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule26);
  // Rule 27
  FuzzyRuleAntecedent *ifDistM_AngP = new FuzzyRuleAntecedent();
  ifDistM_AngP->joinWithAND(dist_M, ang_P);
  FuzzyRule *fuzzyRule27 = new FuzzyRule(27, ifDistM_AngP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule27);
  // Rule 28
  FuzzyRuleAntecedent *ifDistM_AngBP = new FuzzyRuleAntecedent();
  ifDistM_AngBP->joinWithAND(dist_M, ang_BP);
  FuzzyRule *fuzzyRule28 = new FuzzyRule(28, ifDistM_AngBP, thenLeftVB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule28);
  // Rule 29
  FuzzyRuleAntecedent *ifDistSF_AngBN = new FuzzyRuleAntecedent();
  ifDistSF_AngBN->joinWithAND(dist_SF, ang_BN);
  FuzzyRule *fuzzyRule29 = new FuzzyRule(29, ifDistSF_AngBN, thenLeftS_RightVB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule29);
  // Rule 30
  FuzzyRuleAntecedent *ifDistSF_AngN = new FuzzyRuleAntecedent();
  ifDistSF_AngN->joinWithAND(dist_SF, ang_N);
  FuzzyRule *fuzzyRule30 = new FuzzyRule(30, ifDistSF_AngN, thenLeftS_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule30);
  // Rule 31
  FuzzyRuleAntecedent *ifDistSF_AngSN = new FuzzyRuleAntecedent();
  ifDistSF_AngSN->joinWithAND(dist_SF, ang_SN);
  FuzzyRule *fuzzyRule31 = new FuzzyRule(31, ifDistSF_AngSN, thenLeftS_RightVB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule31);
  // Rule 32
  FuzzyRuleAntecedent *ifDistSF_AngZ = new FuzzyRuleAntecedent();
  ifDistSF_AngZ->joinWithAND(dist_SF, ang_Z);
  FuzzyRule *fuzzyRule32 = new FuzzyRule(32, ifDistSF_AngZ, thenLeftB_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule32);
  // Rule 33
  FuzzyRuleAntecedent *ifDistSF_AngSP = new FuzzyRuleAntecedent();
  ifDistSF_AngSP->joinWithAND(dist_SF, ang_SP);
  FuzzyRule *fuzzyRule33 = new FuzzyRule(33, ifDistSF_AngSP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule33);
  // Rule 34
  FuzzyRuleAntecedent *ifDistSF_AngP = new FuzzyRuleAntecedent();
  ifDistSF_AngP->joinWithAND(dist_SF, ang_P);
  FuzzyRule *fuzzyRule34 = new FuzzyRule(34, ifDistSF_AngP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule34);
  // Rule 35
  FuzzyRuleAntecedent *ifDistSF_AngBP = new FuzzyRuleAntecedent();
  ifDistSF_AngBP->joinWithAND(dist_SF, ang_BP);
  FuzzyRule *fuzzyRule35 = new FuzzyRule(35, ifDistSF_AngBP, thenLeftVB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule35);
  // Rule 36
  FuzzyRuleAntecedent *ifDistF_AngBN = new FuzzyRuleAntecedent();
  ifDistF_AngBN->joinWithAND(dist_F, ang_BN);
  FuzzyRule *fuzzyRule36 = new FuzzyRule(36, ifDistF_AngBN, thenLeftS_RightVB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule36);
  // Rule 37
  FuzzyRuleAntecedent *ifDistF_AngN = new FuzzyRuleAntecedent();
  ifDistF_AngN->joinWithAND(dist_F, ang_N);
  FuzzyRule *fuzzyRule37 = new FuzzyRule(37, ifDistF_AngN, thenLeftS_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule37);
  // Rule 38
  FuzzyRuleAntecedent *ifDistF_AngSN = new FuzzyRuleAntecedent();
  ifDistF_AngSN->joinWithAND(dist_F, ang_SN);
  FuzzyRule *fuzzyRule38 = new FuzzyRule(38, ifDistF_AngSN, thenLeftM_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule38);
  // Rule 39
  FuzzyRuleAntecedent *ifDistF_AngZ = new FuzzyRuleAntecedent();
  ifDistF_AngZ->joinWithAND(dist_F, ang_Z);
  FuzzyRule *fuzzyRule39 = new FuzzyRule(39, ifDistF_AngZ, thenLeftB_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule39);
  // Rule 40
  FuzzyRuleAntecedent *ifDistF_AngSP = new FuzzyRuleAntecedent();
  ifDistF_AngSP->joinWithAND(dist_F, ang_SP);
  FuzzyRule *fuzzyRule40 = new FuzzyRule(40, ifDistF_AngSP, thenLeftB_RightM);
  fuzzy_tracking->addFuzzyRule(fuzzyRule40);
  // Rule 41
  FuzzyRuleAntecedent *ifDistF_AngP = new FuzzyRuleAntecedent();
  ifDistF_AngP->joinWithAND(dist_F, ang_P);
  FuzzyRule *fuzzyRule41 = new FuzzyRule(41, ifDistF_AngP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule41);
  // Rule 42
  FuzzyRuleAntecedent *ifDistF_AngBP = new FuzzyRuleAntecedent();
  ifDistF_AngBP->joinWithAND(dist_F, ang_BP);
  FuzzyRule *fuzzyRule42 = new FuzzyRule(42, ifDistF_AngBP, thenLeftVB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule42);
  // Rule 43
  FuzzyRuleAntecedent *ifDistVF_AngBN = new FuzzyRuleAntecedent();
  ifDistVF_AngBN->joinWithAND(dist_VF, ang_BN);
  FuzzyRule *fuzzyRule43 = new FuzzyRule(43, ifDistVF_AngBN, thenLeftS_RightVB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule43);
  // Rule 44
  FuzzyRuleAntecedent *ifDistVF_AngN = new FuzzyRuleAntecedent();
  ifDistVF_AngN->joinWithAND(dist_VF, ang_N);
  FuzzyRule *fuzzyRule44 = new FuzzyRule(44, ifDistVF_AngN, thenLeftS_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule44);
  // Rule 45
  FuzzyRuleAntecedent *ifDistVF_AngSN = new FuzzyRuleAntecedent();
  ifDistVF_AngSN->joinWithAND(dist_VF, ang_SN);
  FuzzyRule *fuzzyRule45 = new FuzzyRule(45, ifDistVF_AngSN, thenLeftS_RightB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule45);
  // Rule 46
  FuzzyRuleAntecedent *ifDistVF_AngZ = new FuzzyRuleAntecedent();
  ifDistVF_AngZ->joinWithAND(dist_VF, ang_Z);
  FuzzyRule *fuzzyRule46 = new FuzzyRule(46, ifDistVF_AngZ, thenLeftVB_RightVB);
  fuzzy_tracking->addFuzzyRule(fuzzyRule46);
  // Rule 47
  FuzzyRuleAntecedent *ifDistVF_AngSP = new FuzzyRuleAntecedent();
  ifDistVF_AngSP->joinWithAND(dist_VF, ang_SP);
  FuzzyRule *fuzzyRule47 = new FuzzyRule(47, ifDistVF_AngSP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule47);
  // Rule 48
  FuzzyRuleAntecedent *ifDistVF_AngP = new FuzzyRuleAntecedent();
  ifDistVF_AngP->joinWithAND(dist_VF, ang_P);
  FuzzyRule *fuzzyRule48 = new FuzzyRule(48, ifDistVF_AngP, thenLeftB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule48);
  // Rule 49
  FuzzyRuleAntecedent *ifDistVF_AngBP = new FuzzyRuleAntecedent();
  ifDistVF_AngBP->joinWithAND(dist_VF, ang_BP);
  FuzzyRule *fuzzyRule49 = new FuzzyRule(49, ifDistVF_AngBP, thenLeftVB_RightS);
  fuzzy_tracking->addFuzzyRule(fuzzyRule49);
}

void uart_receive(){
  while (Serial.available()) {
    dancing_var = Serial.readString();
    if(state == 1){
      setHome = 1;
    }
  }
}

void dancing_base(){
  uart_receive();
  Read_Bluetooth();
  if(dancing_var == "D1"){
    if(state == 0){
      for(int i =0; i<30; i++){
        Read_Bluetooth();
        uart_receive();
        Forward(-120, 120);
        delay(100);
      }
      state = 1;
    }
    else if(state == 1){
      for(int i =0; i<60; i++){
        Read_Bluetooth();
        uart_receive();
        Forward(120, -120);
        delay(100);
      }
      for(int i =0; i<60; i++){
        Read_Bluetooth();
        uart_receive();
        Forward(-120, 120);
        delay(100);
      }
      if(setHome == 1){
        for(int i =0; i<30; i++){
          Read_Bluetooth();
          uart_receive();
          Forward(120,-120);
          delay(100);
          setHome = 0;
        }
      }
    }
  }
  else if(dancing_var=="S"){
    Forward(0,0);
    state = 0;
  }
}


String t = "";
void DOCKING_PROCESS(){
  while (Serial.available()) {
    Read_Bluetooth();
    t = Serial.readString();
//    Serial.println(t);
    
    // Tìm vị trí của dấu phẩy trong chuỗi
    int commaIndex = t.indexOf(",");
    
    // Tách phần giá trị đầu tiên
    String value1String = t.substring(0, commaIndex);
    dock_x1 = value1String.toFloat();
    
    // Tách phần giá trị thứ hai
    String value2String = t.substring(commaIndex + 1);
    dock_z1 = value2String.toFloat();

    if(dock_z1 < 23){
      Backward(80,80);
    }
    else{
      fuzzy_docking->setInput(1, dock_x1);   // fuzzy input (dock_x1)
      fuzzy_docking->setInput(2, dock_z1);   // fuzzy input (dock_z1)
      fuzzy_docking->fuzzify();
      
      //***** defuzzyfication *****//
      int left_motor  = fuzzy_docking->defuzzify(1); // defuzzify fuzzy output 1 (Left motor)
      int right_motor = fuzzy_docking->defuzzify(2); // defuzzify fuzzy output 2 (right motor)
  
  //    Serial.print(left_motor);
  //    Serial.print(" ");
  //    Serial.println(right_motor);
  
      Backward(left_motor, right_motor);
    }
//    
//    // In ra giá trị đã tách
//    Serial.print("dock_x: ");
//    Serial.println(dock_x);
//    Serial.print("dock_z: ");
//    Serial.println(dock_z);

    // Remove the flags and rebuild Serial
//    Serial.end();
//    Serial.begin(9600);
  }   
}

//***** Setup function *****//
void setup() {
  //***** UART DEFAULT *****//
  Serial.begin(9600);
  Serial.setTimeout(10);
  
  //********************* START SETUP UART DRIVER******************************************//
  mySerial.begin(19200);   // set the data rate for the SoftwareSerial port

  //******************** START SETUP BLUETOOTH********************************************//
  Serial3.begin(9600);
  while (!Serial) {}
//  Serial.println("Configuring, please wait...");
  for (int i = 0 ; i < 9 ; i++) {
    Serial3.begin(baudRate[i]);
    String cmd = "AT+BAUD4";
    Serial3.print(cmd);
    Serial3.flush();
    delay(100);
  }
  Serial3.begin(9600);
//  Serial.println("Config done");
  while (!Serial3) {}
//  Serial.println("Enter AT commands:");
  //******************** END SETUP BLUETOOTH*********************************************//

  //********************* SET UP PINS FOR ULTRASONICS SENSOR ******************************//
  pinMode(trig1,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(trig2,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(trig3,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(trig4,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(trig5,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(trig6,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(trig7,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(echo1,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(echo2,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(echo3,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(echo4,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(echo5,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(echo6,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(echo7,INPUT);    // chân echo sẽ nhận tín hiệu

  pinMode(trig_behind1,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(echo_behind1,INPUT);    // chân echo sẽ nhận tín hiệu
  pinMode(trig_behind2,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(echo_behind2,INPUT);    // chân echo sẽ nhận tín hiệu

  //********************* SET UP PINS FOR ENCODER ******************************//
  pinMode(encoderPinLA, INPUT_PULLUP);
  pinMode(encoderPinLB, INPUT_PULLUP);
  pinMode(encoderPinRA, INPUT_PULLUP);
  pinMode(encoderPinRB, INPUT_PULLUP);

  //********************* SET UP INTERRUPTS FOR ENCODER ******************************//
  attachInterrupt(digitalPinToInterrupt(2),   doEncoderRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3),   doEncoderRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20),  doEncoderLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21),  doEncoderLB, CHANGE);

  //********************* SET UP STOP ******************************//
  pinMode(13, INPUT_PULLUP);

  //*********************************** SETUP FUZZY ***********************************//
  Fuzzy_Obstacle();
  Fuzzy_Tracking();
  Fuzzy_Docking();
}

//***** While True *****//
void loop() {
  present_time = millis();
  delta_time = (double) present_time - previous_time;
  if(delta_time >= 100){
    previous_time = present_time;
    if(ctrlMode <= 1){
      resert_value();
      
      //***** Code Bluetooth *****//
      if (Serial3.available()>0){
        value = Serial3.read();
//        Serial.println(value);
        switch(value){
          case 'F':
            Forward(100,100);
            break;
          case 'D':
            Forward(-100,-100);
            break;
          case 'L':
            Turn_Left();
            break;
          case 'R':
            Turn_Right();
            break;
          case 'A':
            Forward_Right();
            break;
          case 'B':
            Forward_Left();
            break;
          case 'C':
            Backward_Left();
            break;
          case 'E':
            Backward_Right();
            break;
          case 'M':
            ctrlMode = 1;
            break;
          case 'U':
            ctrlMode = 2;
            break;
          case 'O':
            ctrlMode = 3;
            break;
          case 'S':
            Stop();
            break;
        }
      }
    }
    else if(ctrlMode == 2){
      Read_Bluetooth();

      // Localization
      odometry();
      
      //***** Read distance from 7 sensor *****//
      distance1 = distance_func(trig1, echo1);
      distance2 = distance_func(trig2, echo2);
      distance3 = distance_func(trig3, echo3);
      distance4 = distance_func(trig4, echo4);
      distance5 = distance_func(trig5, echo5);
      distance6 = distance_func(trig6, echo6);
      distance7 = distance_func(trig7, echo7);
  
      //***** Reduce 7 into 3 sensors *****//
      DLeft   = findNear(distance1, distance2, 1000);
      DCenter = findNear(distance3, distance4, distance5);
      DRight  = findNear(distance6, distance7, 1000);
      
      if((DLeft <= 55 || DCenter <= 55 || DRight <= 55) && d_actual < d_target){
        fuzzy_obstacle->setInput(1, DLeft);    // fuzzy input (left sensor)
        fuzzy_obstacle->setInput(2, DCenter);  // fuzzy input (center sensor)
        fuzzy_obstacle->setInput(3, DRight);   // fuzzy input (right sensor)
        fuzzy_obstacle->fuzzify();
        
        //***** defuzzyfication *****//
        int left_motor  = fuzzy_obstacle->defuzzify(1); // defuzzify fuzzy output 1 (Left motor)
        int right_motor = fuzzy_obstacle->defuzzify(2); // defuzzify fuzzy output 2 (right motor)
        
        //***** Control motor *****//
        Forward(left_motor, right_motor);
  
        //***** Display *****//
//        Serial.print(" ");
//        Serial.print(distance1);
//        Serial.print(" ");
//        Serial.print(distance2);
//        Serial.print(" ");
//        Serial.print(distance3);
//        Serial.print(" ");
//        Serial.print(distance4);
//        Serial.print(" ");
//        Serial.print(distance5);
//        Serial.print(" ");
//        Serial.print(distance6);
//        Serial.print(" ");
//        Serial.println(distance7);
//      
//        Serial.print("DL: ");
//        Serial.print(DLeft);
//        Serial.print(" DC: ");
//        Serial.print(DCenter);  
//        Serial.print(" DR: ");
//        Serial.println(DRight); 
//      
//        Serial.print("left: ");
//        Serial.print(left_motor);
//        Serial.print(" right: ");
//        Serial.println(right_motor); 
//      
//        Serial.println("   ");
      }
      else if (d_actual < d_target){
        phi_temp = abs(phi_target - atan2(y,x));
        d_actual = (float)(cos(phi_temp))*sqrt(pow(x,2) + pow(y,2));
        phi_actual = phi_degree - (atan2((Yd-y), (Xd-x))*180)/PI;
  
        fuzzy_tracking->setInput(1, d_actual);   // fuzzy input (distance)
        fuzzy_tracking->setInput(2, phi_actual);   // fuzzy input (angle)
        fuzzy_tracking->fuzzify();
        
        //***** defuzzyfication *****//
        int left_motor  = fuzzy_tracking->defuzzify(1); // defuzzify fuzzy output 1 (Left motor)
        int right_motor = fuzzy_tracking->defuzzify(2); // defuzzify fuzzy output 2 (right motor)

        Forward(left_motor, right_motor);
        
      
//        Serial.print("Phi_Actual ");  Serial.println(phi_actual);
//        Serial.print("D_Actual ");    Serial.println(d_actual);
//        Serial.print("Vleft ");       Serial.println(left_motor);
//        Serial.print("VRight");       Serial.println(right_motor);
      }
      else{
        Forward(0, 0);
      }
    }
    else if(ctrlMode == 3){
      Read_Bluetooth();
//      while(!Serial.available() && ctrlMode ==3){Read_Bluetooth();}
//      data_docking = Serial.read();
//      //Serial.print(data_docking);
//
//      distance_behind1 = distance_func(trig_behind1, echo_behind1);
//      distance_behind2 = distance_func(trig_behind2, echo_behind2);
//
//      if(!digitalRead(13) || distance_behind1 <10 || distance_behind2 < 10){
//        Serial.println("S");
//      }
//      else{
//        if(data_docking == '1'){
//          Backward_Left();
//        }
//        else if(data_docking == '2'){
//          Backward_Right();
//        }
//        else if(data_docking == '3'){
//          Backward(-100, -100);
//        }
//        else if(data_docking == '5'){
//          Forward(-100,-100);
//        }
//        else if(data_docking == '6'){
//          Stop();
//        }
//      }

        if(digitalRead(13)==0){
          Backward(0,0);
          Serial.println("S");
        }
        else{
          DOCKING_PROCESS();
        }
//        dancing_base();

      }
    }
      //Serial.println(ctrlMode);
  }

   

//************ LEFT WHEEL ************//
void doEncoderLA(){
  if(digitalRead(encoderPinLA) != digitalRead(encoderPinLB)){
    pulseL++;
  }
  else{
    pulseL--;
  }
}

void doEncoderLB(){
  if(digitalRead(encoderPinLA) == digitalRead(encoderPinLB)){
    pulseL++;
  }
  else{
    pulseL--;
  }
}

//************ RIGHT WHEEL ************//
void doEncoderRA(){
  if(digitalRead(encoderPinRA) != digitalRead(encoderPinRB)){
    pulseR++;
  }
  else{
    pulseR--;
  }
}

void doEncoderRB(){
  if(digitalRead(encoderPinRA) == digitalRead(encoderPinRB)){
    pulseR++;
  }
  else{
    pulseR--;
  }
}
