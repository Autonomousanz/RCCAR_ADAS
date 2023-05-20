#include <Servo.h> //define the servo library
Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc
int steering = 90, throttle = 90; //defining global variables to use later

//////////////*******steering control**********/////////////////
//int StrD = 0;

float distance1, distance2, duration1, duration2, z, sides;
const float echo1 = 8;
const float trig1 = 7;
const float echo2 = 9;
const float trig2 = 6;
float Steervalue;

int thr;

#define kp 0.8
#define ki 0.0001
#define kd 2

float priError = 0;
float toError = 0;
//////////////*******steering control end**********/////////////////

//////////////*******throttle control**********/////////////////
int setP = 35;

float distance, duration;
const float echo = 4;
const float trig = 5;


#define kpt 1.2
#define kit 0
#define kdt 6

float preE = 0;
float toE = 0;
//////////////*******throttle control end**********/////////////////

void setup() {
  Serial.begin(115200); //start serial connection. Uncomment for PC
  ssm.attach(10);    //define that ssm is connected at pin 10
  esc.attach(11);     //define that esc is connected at pin 11
  //////////////////*************////////////////Steering start
  pinMode(echo1, INPUT);
  pinMode(trig1, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig2, OUTPUT);
  /////////////////**************////////////////Steering end
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  setVehicle(90, 90);
  delay(500);
  ////////////////////////////////throttle end
}

////*************************************************Setup end****************************************************//



void loop() {
  PID();   //Steering PID
  PIDt();  //Throttle PID

  if (thr > 90) {
    setVehicle(Steervalue, 95);
    delay(50);
    setVehicle(Steervalue, 90);
    delayMicroseconds(10);
  }
  else if (thr == 90) {
    setVehicle(Steervalue, 90);
  }
  else setVehicle(Steervalue, 75);
  // delay(max(abs(thr - 90) * 10, 40));
  delay(60);
  setVehicle(Steervalue, 90);
  delayMicroseconds(10);
  Serial.print(Steervalue);
  Serial.print("\t");

}

//////Steering side sensor reading/////////////////
float d_left() {
  digitalWrite(trig1, LOW);
  //digitalWrite(light, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig1, LOW);
  duration1 = pulseIn(echo1, HIGH, 10000);
  if (duration1 != 0)
    distance1 = ((0.176291554 * duration1) - 12.52197797) / 10;
  else return 1000;
  return distance1;
}
float d_right() {
  digitalWrite(trig2, LOW);
  //digitalWrite(light, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig2, LOW);
  duration2 = pulseIn(echo2, HIGH, 10000);
  if (duration2 != 0)
    distance2 = ((0.176291554 * duration2) - 12.52197797) / 10;
  else return 1000;
  return distance2;
}

//////Throttle front sensor reading/////////////////
float dist() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration * 0.03435) / 2;
  //  delay(10);
  return distance;
}
/////////////Actual distance calc for steering//////
float dis() {
  float dl = d_left();
  float dr = d_right();
  if (dl != 1000 and dr != 1000)
    return max(min(dr - dl, 25), -25);
  else if (dl == 1000 and dr == 1000)
    return 0;
  else if (dl != 1000)
    return max(25 - dl, 0);
  else return -max(25 - dr, 0);
}


//float side() {
//  float dl = d_left();
//  float dr = d_right();
//  if (dl > dr)
//    return -1;
//  else return 1;
//}
////////////////////////end////////


/////////////////////PID for throttle/////////////////
#define minthrottle 77
void PIDt() {

  float Errort = setP - dist();
  float Pvalue = Errort * kpt;
  float Ivalue = toE * kit;
  float Dvalue = (Errort - preE) * kdt;

  float PIDtvalue = Pvalue + Ivalue + Dvalue;
  preE = Errort;
  toE += Errort;
  //  Serial.println(PIDtvalue);
  int Trotvalue = (int)PIDtvalue;
  if (Trotvalue >= 0)
    thr = map(Trotvalue, 0, 10, 90, minthrottle);
  else thr = map(Trotvalue, 0, -200, 90, 95);

  thr = max(min(thr, 95), minthrottle);
  //  setVehicle(90, thr);
  //  delay(40);
  //  setVehicle(90, 90);
  //  delay(10);
  //Serial.println(thr);
}


////////////////////PID for Steering////////////
void PID() {
  //  float error = StrD - (dis() * side());
  float error = dis();
  float Pvalue = error * kp;
  float Ivalue = toError * ki;
  float Dvalue = (error - priError) * kd;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  toError = priError;
  priError = error;
  Steervalue = max(min(PIDvalue * 5, 250), -250);

  Steervalue = map(Steervalue, -250, 250, 50, 130);

  //  Serial.print(side());
  //  Serial.print("\t\t");
  //  Serial.print(dis());
  //  Serial.print("\t\t");
  //  Serial.print(d_left());
  //  Serial.print("\t\t");
  //  Serial.print(d_right());
  //  Serial.print("\t\t");
  //  Serial.print(error);
  //  Serial.print("\t\t");
  //  Serial.print(Steervalue);
  //  Serial.println();
  //  setVehicle(Steervalue, 95);
  //  delay(50);
  //  setVehicle(Steervalue, 90);
  //  delay(25);
  //  Serial.print(Steervalue);
  //  Serial.print("\t\t");
  //  Serial.print(dis());
  //  Serial.println();
}

///////////////////////////////end//////////////////////////////////////
//********************** Vehicle Control **********************//
//***************** Do not change below part *****************//
void setVehicle(int s, int v)
{
  s = min(max(0, s), 180); //saturate steering command
  v = min(max(70, v), 110); //saturate velocity command
  ssm.write(s); //write steering value to steering servo
  esc.write(v); //write velocity value to the ESC unit
}
//***************** Control end *****************//
