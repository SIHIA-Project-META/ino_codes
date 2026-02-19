#ifndef PIDCode_h
#define PIDCode_h

#include <Arduino.h>
#include <AutoPID.h>

void Input(double Xi, double Yi, double Zi, double Hi);
void Setpoint(double Xs, double Ys, double Zs, double Hs);

void Config(double Xp, double Xd, double Yp, double Yd, 
            double Zp, double Zd, double Hp, double Hd);

void RunPID(bool X, bool Y, bool Z, bool H);
void SetPeriod(unsigned long Period);

double GetM1();
double GetM2();
double GetM3();
double GetM4();

#endif