#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50
int angle[16];
int servoref[16] = {84, 90, 90, 0, 90, 90, 85, 0, 96, 94, 94, 0, 88, 88, 0, 96};
void holdposition();
void IK();
int pitch;
float c[4], d[4], x[4], z[4];

//Inverse Kinematics Codes For Quadruped Robots by Nic Aqueous//
//https://www.youtube.com/c/NicAqueous for full code explanations//

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  c[0] = c[1] = c[2] = c[3] = 150;
  x[0] = x[1] = x[2] = x[3] = 0;
  pitch = 0;
  Serial.println("ready for command");
}


void loop() {
  if (Serial.available() == 0)
  { IK();
    holdposition();
  }
  else
  {
    char command = Serial.read();
    if (command == 'w')
    {
      if (d[0] < 190 && d[1] < 190 && d[2] < 190 && d[3] < 190)
      {
        c[0] += 2;
        c[1] += 2;
        c[2] += 2;
        c[3] += 2;
        IK();
        holdposition();
      }
    }
    if (command == 's')
    {
      if (d[0] > 118 && d[1] > 118 && d[2] > 118 && d[3] > 118)
      {
        c[0] -= 2;
        c[1] -= 2;
        c[2] -= 2;
        c[3] -= 2;
        IK();
        holdposition();
      }
    }
    if (command == 'n')
    {
      c[0] = c[1] = c[2] = c[3] = 140;
      x[0] = x[1] = x[2] = x[3] = 0;
      pitch = 0;
    }
    if (command == 'p')
    {
      for (d[0] > 118, d[1] > 118, d[2] > 118, d[3] > 118; d[0] < 190, d[1] < 190, d[2] < 190, d[3] < 190; c[0] += 2, c[1] += 2, c[2] += 2, c[3] += 2)
      {
        IK();
        holdposition();
      }
      for (d[0] = d[1] = d[2] = d[3] = 190; d[0] > 118, d[1] > 118, d[2] > 118, d[3] > 118; c[0] -= 2, c[1] -= 2, c[2] -= 2, c[3] -= 2)
      {
        IK();
        holdposition();
      }
    }
    if (command == 'b')
    {
      if (x[0] < 40 && x[1] < 40 && x[2] < 40 && x[3] < 40)
      {
        x[0] += 2;
        x[1] += 2;
        x[2] += 2;
        x[3] += 2;
        IK();
        holdposition();
      }
    }
    if (command == 'f')
    {
      if (x[0] > -40 && x[1] > -40 && x[2] > -40 && x[3] > -40)
      {
        x[0] -= 2;
        x[1] -= 2;
        x[2] -= 2;
        x[3] -= 2;
        IK();
        holdposition();
      }
    }
    if (command == 'a')
    {
      for (x[0] < 30, x[1] < 30, x[2] < 30, x[3] < 30; x[0] > -40, x[1] > -40, x[2] > -40, x[3] > -40; x[0] -= 2, x[1] -= 2, x[2] -= 2, x[3] -= 2)
      {
        IK();
        holdposition();
      }
      delay(10);
      for (d[0] > 118, d[1] > 118, d[2] > 118, d[3] > 118; d[0] < 190, d[1] < 190, d[2] < 190, d[3] < 190; c[0] += 2, c[1] += 2, c[2] += 2, c[3] += 2)
      {
        IK();
        holdposition();
      }
      delay(10);
      for (x[0] > -40, x[1] > -40, x[2] > -40, x[3] > -40; x[0] < 30, x[1] < 30, x[2] < 30, x[3] < 30; x[0] += 2, x[1] += 2, x[2] += 2, x[3] += 2)
      {
        IK();
        holdposition();
      }
      delay(10);
      for (d[0] = d[1] = d[2] = d[3] = 190; d[0] > 118, d[1] > 118, d[2] > 118, d[3] > 118; c[0] -= 2, c[1] -= 2, c[2] -= 2, c[3] -= 2)
      {
        IK();
        holdposition();
      }
      delay(10);
      for (x[0] < 30, x[1] < 30, x[2] < 30, x[3] < 30; x[0] >= 0, x[1] >= 0, x[2] >= 0, x[3] >= 0; x[0] -= 2, x[1] -= 2, x[2] -= 2, x[3] -= 2)
      {
        IK();
        holdposition();
      }
    }
    if (command == 'x')
    {
      for (x[0] < 30, x[1] < 30, x[2] < 30, x[3] < 30; x[0] > -40, x[1] > -40, x[2] > -40, x[3] > -40; x[0] -= 2, x[1] -= 2, x[2] -= 2, x[3] -= 2)
      {
        IK();
        holdposition();
      }
      delay(10);
      for (x[0] > -40, x[1] > -40, x[2] > -40, x[3] > -40; x[0] < 30, x[1] < 30, x[2] < 30, x[3] < 30; x[0] += 2, x[1] += 2, x[2] += 2, x[3] += 2)
      {
        IK();
        holdposition();
      }
    }
    if (command == 'z')
    {
      for (d[0] > 118, d[1] > 118, d[2] > 118, d[3] > 118; d[0] < 150, d[1] < 150, d[2] < 150, d[3] < 150; c[0] += 2, c[1] += 2, c[2] += 2, c[3] += 2)
      {
        IK();
        holdposition();
      }
      delay(10);
      for (d[0] = d[1] = d[2] = d[3] = 150; d[0] > 118, d[1] > 118, d[2] > 118, d[3] > 118; c[0] -= 2, c[1] -= 2, c[2] -= 2, c[3] -= 2)
      {
        IK();
        holdposition();
      }
    }
    if (command == 'm')
    {
      for (x[0] = 0, x[3] = 0; x[0] <= 60, x[3] <= 60; x[0] += 3, x[3] += 3)
      {
        z[0] = sqrt(900 - pow((x[0] - 30), 2));
        z[3] = sqrt(900 - pow((x[3] - 30), 2));
        IK();
        holdposition();
      }
      for (x[0] = 60, x[3] = 60; x[0] >= 0, x[3] >= 0 ; x[0] -= 3, x[3] -= 3)
      {
        IK();
        holdposition();
      }
      for (x[1] = 0, x[2] = 0; x[1] <= 60, x[2] <= 60; x[1] += 3, x[2] += 3)
      {
        z[1] = sqrt(900 - pow((x[1] - 30), 2));
        z[2] = sqrt(900 - pow((x[2] - 30), 2));
        IK();
        holdposition();
      }
      for (x[1] = 60, x[2] = 60; x[1] >= 0, x[2] >= 0 ; x[1] -= 3, x[2] -= 3)
      {
        IK();
        holdposition();
      }
    }
    if (command == 'l')
    {
      for (x[0] = 0, x[3] = 0; x[0] <= 60, x[3] <= 60; x[0] += 3, x[3] += 3)
      {
        z[0] = 40* (sqrt(1 - (pow((x[0] - 30), 2)/900)));
        z[3] = 40* (sqrt(1 - (pow((x[3] - 30), 2)/900)));
        IK();
        holdposition();
        delay(5);
      }
      for (x[0] = 60, x[3] = 60; x[0] >= 0, x[3] >= 0 ; x[0] -= 3, x[3] -= 3)
      {
        IK();
        holdposition();
        delay(5);
      }
      for (x[1] = 0, x[2] = 0; x[1] <= 60, x[2] <= 60; x[1] += 3, x[2] += 3)
      {
        z[1] = 40* (sqrt(1 - (pow((x[1] - 30), 2)/900)));
        z[2] = 40* (sqrt(1 - (pow((x[2] - 30), 2)/900)));
        IK();
        holdposition();
        delay(5);
      }
      for (x[1] = 60, x[2] = 60; x[1] >= 0, x[2] >= 0 ; x[1] -= 3, x[2] -= 3)
      {
        IK();
        holdposition();
        delay(5);
      }
    }
    if (command == 'e')
    {
      pitch++;
      IK();
      holdposition();
      delay(15);
    }
    if (command == 'd')
    {
      pitch--;
      IK();
      holdposition();
      delay(15);
    }
    if (command == 'r')
    {
      for (pitch > -15; pitch < 15; pitch++)
      {
        IK();
        holdposition();
      }
      delay(15);
      for (pitch < 15; pitch > -15; pitch--)
      {
        IK();
        holdposition();
      }
      delay(15);
      for (pitch > -15; pitch <= 0; pitch++)
      {
        IK();
        holdposition();
      }
    }
  }
}
void holdposition()
{
  int microseconds[16];
  for (int i = 0; i < 16; i++)
  {
    int anglemap = angle[i];
    int microsecondsmap = map(anglemap, 0, 180, 500, 2500);
    microseconds[i] = microsecondsmap;
    pwm.writeMicroseconds(i, microseconds[i]);
  }
}
void IK()
{
  float A[4], B[4], C[4], D[4], s[4], thetaX[4];
  int a = 80;
  float b = 140.251;
  for (int n = 0; n < 4; n++)
  {
    if (pitch >= 0)
    {
      if (n == 1 || n == 3)
      {
        thetaX[n] = atan(x[n]+ 96.425*(1 - cos((pitch * PI) / 180)) / c[n]);
        d[n] = ((c[n] - z[n]) + 96.425 * sin((pitch*PI)/180)) / cos(thetaX[n]);
      }
      if (n == 0 || n == 2)
      {     
        thetaX[n] = atan(x[n]- 96.425*(1 - cos((pitch*PI)/180)) / c[n]);
        d[n] = ((c[n] - z[n]) - 96.425 * sin((pitch*PI)/180)) / cos(thetaX[n]);
      }
    }
    else
    {
      if (n == 0 || n == 2)
      {
        x[n] + 96.425*(1 - cos((pitch*PI)/180));
        thetaX[n] = atan(x[n] / c[n]);
        d[n] = ((c[n] - z[n]) - 96.425 * sin((pitch*PI)/180)) / cos(thetaX[n]);
      }
      if (n == 1 || n == 3)
      {
        s[n] = x[n] - 96.425*(1 - cos((pitch*PI)/180));
        thetaX[n] = atan(s[n] / c[n]);
        d[n] = ((c[n] - z[n]) + 96.425 * sin((pitch*PI)/180)) / cos(thetaX[n]);
      }
    }
  }
  for (int n = 0; n < 4; n++)
  {
    A[n] = acos((pow(b, 2) + pow(d[n], 2) - pow(a, 2)) / (2 * b * d[n]));
    B[n] = acos((pow(a, 2) + pow(d[n], 2) - pow(b, 2)) / (2 * a * d[n]));
    C[n] = acos((pow(a, 2) + pow(b, 2) - pow(d[n], 2)) / (2 * a * b));
    D[n] = 90 - ((C[n] * 180) / PI);
  }
  for (int i = 0; i < 16; i++)
  {
    if (i == 0)
    {
      if (C[0] >= PI / 2)
      {
        angle[i] = servoref[i] + abs(D[0]);
      }
      else
      {
        angle[i] = servoref[i] - abs(D[0]);
      }
    }
    else if (i == 1)
    {
      angle[i] = servoref[i] - ((B[0] * 180) / PI) + ((thetaX[0] * 180) / PI);
    }
    else if (i == 2)
    {
      angle[i] = servoref[i];
    }
    else if (i == 4)
    {
      if (C[1] >= PI / 2)
      {
        angle[i] = servoref[i] + abs(D[1]);
      }
      else
      {
        angle[i] = servoref[i] - abs(D[1]);
      }
    }
    else if (i == 5)
    {
      angle[i] = servoref[i] - ((B[1] * 180) / PI) + ((thetaX[1] * 180) / PI);
    }
    else if (i == 6)
    {
      angle[i] = servoref[i];
    }
    else if (i == 8)
    {
      angle[i] = servoref[i];
    }
    else if (i == 9)
    {
      angle[i] = servoref[i] + ((B[3] * 180) / PI) - ((thetaX[3] * 180) / PI);
    }
    else if (i == 10)
    {
      if (C[3] >= PI / 2)
      {
        angle[i] = servoref[i] - abs(D[3]);
      }
      else
      {
        angle[i] = servoref[i] + abs(D[3]);
      }
    }
    else if (i == 12)
    {
      angle[i] = servoref[i];
    }
    else if (i == 13)
    {
      angle[i] = servoref[i] + ((B[2] * 180) / PI) - ((thetaX[2] * 180) / PI);
    }
    else if (i == 15)
    {
      if (C[2] >= PI / 2)
      {
        angle[i] = servoref[i] - abs(D[2]);
      }
      else
      {
        angle[i] = servoref[i] + abs(D[2]);
      }
    }
    else
    {
      angle[i] = servoref[i];
    }
  }
}
