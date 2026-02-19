#include "PIDCode.h"

#define g 9.80665

//Tem que colocar o Ki, mas pra não ter integrativo é só colocar 0
#define Ki 0.0

double Xinput, Xsetpoint, Xoutput;
double Yinput, Ysetpoint, Youtput;
double Zinput, Zsetpoint, Zoutput;
double Hinput, Hsetpoint, Houtput;

double XKp, XKd;
double YKp, YKd;
double ZKp, ZKd;
double HKp, HKd;

//Unidade em ms
unsigned long PIDPeriod;

//Porcentagem base dos motores
double BaseThrottle = 55.00;
/*
    Frente
M1     X   M2
   \   |   /
    \  |  /
<------Z------> Y
    /  |  \
   /   |   \
 M3         M4
*/
//Porcentagem final de cada motor
double M1, M2, M3, M4;

//O output vai ser +-10% para rotação e +-15% para altura, por enquanto são valores de chute, não sei quais seriam os melhores
AutoPID XAnglePD(&Xinput, &Xsetpoint, &Xoutput, -10.00, 10.00, 0.0, Ki, 0.0);

AutoPID YAnglePD(&Yinput, &Ysetpoint, &Youtput, -10.00, 10.00, 0.0, Ki, 0.0);

AutoPID ZAnglePD(&Zinput, &Zsetpoint, &Zoutput, -10.00, 10.00, 0.0, Ki, 0.0);

AutoPID HeightPD(&Hinput, &Hsetpoint, &Houtput, -15.00, 15.00, 0.0, Ki, 0.0);

void Input(double Xi, double Yi, double Zi, double Hi) {
  
  Xinput = Xi;
  Yinput = Yi;
  Zinput = Zi;

  //Assumindo que Hi seja a velocidade sem a conversão em relação a gravidade
  Hinput = Hi/g;

}

void Setpoint(double Xs, double Ys, double Zs, double Hs) {

  //Assumindo que o input já esteja na unidade de medida correta
  Xsetpoint = Xs;
  Ysetpoint = Ys;
  Zsetpoint = Zs;
  Hsetpoint = Hs;

}

void Config(double Xp, double Xd, double Yp, double Yd, double Zp, double Zd, double Hp, double Hd) {

  XKp = Xp;
  XKd = Xd;

  XAnglePD.setGains(XKp, Ki, XKd);
  
  YKp = Yp;
  YKd = Yd;

  YAnglePD.setGains(YKp, Ki, YKd);
  
  ZKp = Zp;
  ZKd = Zd;

  ZAnglePD.setGains(ZKp, Ki, ZKd);

  HKp = Hp;
  HKd = Hd;

  HeightPD.setGains(HKp, Ki, HKd);

}

void SetPeriod(unsigned long Period) {

  PIDPeriod = Period;

  XAnglePD.setTimeStep(PIDPeriod);
  YAnglePD.setTimeStep(PIDPeriod);
  ZAnglePD.setTimeStep(PIDPeriod);
  HeightPD.setTimeStep(PIDPeriod);

}

void RunPID(bool X, bool Y, bool Z, bool H) {

  if (X) XAnglePD.run();
  if (Y) YAnglePD.run();
  if (Z) ZAnglePD.run();
  if (H) HeightPD.run();

}

double GetM1() {

  M1 = BaseThrottle + Xoutput - Youtput - Zoutput + Houtput;

  return M1;
}

double GetM2() {

  M2 = BaseThrottle - Xoutput - Youtput + Zoutput + Houtput;

  return M2;
}

double GetM3() {

  M3 = BaseThrottle + Xoutput + Youtput + Zoutput + Houtput;

  return M3;
}

double GetM4() {

  M4 = BaseThrottle - Xoutput + Youtput - Zoutput + Houtput;

  return M4;
}