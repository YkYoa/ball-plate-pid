#include <TouchScreen.h>
#include "BallPositionFilter.h"
#include <TimerOne.h>
#include "PID_v1.h"


//== == == == == == == == == == == == == == == == == == == ==

#define encoder1PinA 2  // Khai báo chân đọc channel B
#define encoder1PinB 3  // Khai báo chân đọc channel A

#define encoder2PinA 18
#define encoder2PinB 19

#define encoder3PinA 20
#define encoder3PinB 21


volatile double encoder1Count = 0;  // Khai báo biến đếm xung của encoder 0
volatile double encoder2Count = 0;
volatile double encoder3Count = 0;

//== == == == == == == == == == == == == == == == == == == ==

#define YP A2  // must be an analog pin, use "An" notation!
#define XM A1  // must be an analog pin, use "An" notation!
#define YM A0  // can be a digital pin
#define XP A3  // can be a digital pin

// Resistance between X+ and X- is 330 ohms
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 330);
BallPositionFilter ballPosition = BallPositionFilter(&ts);

//== == == == == == == == == == == == == == == == == == == ==

#define PWM_1 4
#define DC1_LEN 5
#define DC1_XUONG 6

#define PWM_2 10
#define DC2_LEN 11
#define DC2_XUONG 12

#define PWM_3 7
#define DC3_LEN 8
#define DC3_XUONG 9

//== == == == == == == == == == == == == == == == == == == ==
volatile double setPoint_X = 0;
volatile double setPoint_Y1 = 0;
volatile double setPoint_Y2 = 0;

volatile double x = 0;
volatile double y1 = 0;
volatile double y2 = 0;

volatile double t1 = 0;
volatile double t2 = 0;
volatile double t3 = 0;

volatile double u1 = 0;
volatile double u2 = 0;
volatile double u3 = 0;

volatile double Output1 = 0;
volatile double Output2 = 0;
volatile double Output3 = 0;

//== == == == == == == == == == == == == == == == == == == ==

#define CENTRE_X 500
#define CENTRE_Y 525
volatile double setPointX = CENTRE_X, setPointY = CENTRE_Y;
int mode = 1;
int setPointDegrees = 0;
int setPointDegrees_1 = 0;


//== == == == == == == == == == == == == == == == == == == ==

double kp_x = 95;
double kd_x = 100;

double kp_1 = 120;
double kd_1 = 25;

double kp_y1 = 95;
double kd_y1 = 100;

double kp_2 = 120;
double kd_2 = 25;

double kp_y2 = 95;
double kd_y2 = 100;

double kp_3 = 120;
double kd_3 = 25;

PID pid_X(&x, &u1, &setPoint_X, kp_x, 0, kd_x, DIRECT);

PID pid_1(&t1, &Output1, &u1, kp_1, 0, kd_1, DIRECT);

PID pid_Y1(&y1, &u2, &setPoint_Y1, kp_y1, 0, kd_y1, DIRECT);

PID pid_2(&t2, &Output2, &u2, kp_2, 0, kd_2, DIRECT);

PID pid_Y2(&y2, &u3, &setPoint_Y2, kp_y2, 0, kd_y2, DIRECT);

PID pid_3(&t3, &Output3, &u3, kp_3, 0 , kd_3, DIRECT);

void setup() {

  Serial.begin(115200);

  //== == == == == == == == == == == == == == == == == == == ==

  //interrupt
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);

  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  pinMode(encoder3PinA, INPUT_PULLUP);
  pinMode(encoder3PinB, INPUT_PULLUP);

  attachInterrupt(0, doencoder1A, CHANGE);
  attachInterrupt(1, doencoder1B, CHANGE);

  attachInterrupt(5, doencoder2A, CHANGE);
  attachInterrupt(4, doencoder2B, CHANGE);

  attachInterrupt(3, doencoder3A, CHANGE);
  attachInterrupt(2, doencoder3B, CHANGE);

  //== == == == == == == == == == == == == == == == == == == ==

  analogWrite(PWM_1, 0);
  analogWrite(PWM_2, 0);
  analogWrite(PWM_3, 0);

  //== == == == == == == == == == == == == == == == == == == ==
  pid_X.SetSampleTime(100);
  pid_X.SetOutputLimits(-0.2, 0.05);
  pid_X.SetMode(AUTOMATIC);

  pid_1.SetSampleTime(100);
  pid_1.SetOutputLimits(-12, 12);
  pid_1.SetMode(AUTOMATIC);

  pid_Y1.SetSampleTime(100);
  pid_Y1.SetOutputLimits(-0.2, 0.05);
  pid_Y1.SetMode(AUTOMATIC);

  pid_2.SetSampleTime(100);
  pid_2.SetOutputLimits(-12, 12);
  pid_2.SetMode(AUTOMATIC);

  pid_Y2.SetSampleTime(100);
  pid_Y2.SetOutputLimits(-0.05, 0.05);
  pid_Y2.SetMode(AUTOMATIC);

  pid_3.SetSampleTime(100);
  pid_3.SetOutputLimits(-12, 12);
  pid_3.SetMode(AUTOMATIC);



  Timer1.initialize(10000);  //don vi us
  Timer1.attachInterrupt(timer_interrupt);
}



void loop() {
  pid_X.Compute();
  pid_Y1.Compute();
  pid_Y2.Compute();

  pid_1.Compute();
  pid_2.Compute();
  pid_3.Compute();


  computeSetPoint(mode);
  TSPoint p = ballPosition.getPoint();
  // Assume the ball is removed if we have 10 contiguous missed readings
  if (ballPosition.isPresent()) {
#ifdef RAW_POSITION_DEBUG
    // Serial.print(", ");
#endif
    x = (p.x - CENTRE_X) * 0.023 / 100;
    y1 = 0.866 * (p.y - CENTRE_Y) * 0.028 / 100 - (p.x - CENTRE_X) * 0.023 * 0.5 / 100;
    y2 = -0.866 * (p.y - CENTRE_Y) * 0.028 / 100 - (p.x - CENTRE_X) * 0.023 * 0.5 / 100;

    setPoint_X = (setPointX - CENTRE_X) * 0.023 / 100;
    setPoint_Y1 = 0.866 * (setPointY - CENTRE_Y) * 0.028 / 100 - (setPointX - CENTRE_X) * 0.023 * 0.5 / 100;
    setPoint_Y2 = -0.866 * (setPointY - CENTRE_Y) * 0.028 / 100 - (setPointX - CENTRE_X) * 0.023 * 0.5 / 100;


    Serial.print("fX:");
    Serial.print(x);
    Serial.print(", fY1:");
    Serial.print(y1);
    Serial.print(", fY2:");
    Serial.print(' ');
    Serial.print(y2);
    Serial.print(' ');
    Serial.print(t1);
    Serial.print(' ');
    Serial.print(t2);
    Serial.print(' ');
    Serial.print(t3);
    Serial.print(' ');
    Serial.print(Output1);
    Serial.print(' ');
    Serial.print(Output2);
    Serial.print(' ');
    Serial.print(Output3);
    Serial.print(' ');
    Serial.print(u1);
    Serial.print(' ');
    Serial.print(u2);
    Serial.print(' ');
    Serial.print(u3);
    // Serial.prin2(' ');
    // Serial.print(setPointX);
    // Serial.print(' ');
    // Serial.print(setPointY);
    Serial.println();
  }
}
