// Biblioteca de uso do IMU
#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define SDA 14
#define SCL 15
#define AMOSTRAGEM 2000
#define ALPHA 0.995f   // Ideal para 1000Hz

// Filtro passa-alta para Yaw
float yaw_hp = 0;
float yaw_prev = 0;
float hp_alpha = 0.0f;

int16_t dados[7];

float ca;
float cg;

float bgx = 0;
float bgy = 0;
float bgz = 0;

float bax = 0;
float bay = 0;
float baz = 0;

bool bias_reg = 0;

float pgx = 0;
float pgy = 0;
float pgz = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;

bool imu_init(int a, int g) {

  Wire.begin(SDA, SCL);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  // Gyro config
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(g << 3);
  Wire.endTransmission(true);

  // Acc config
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(a << 3);
  Wire.endTransmission(true);

  // DLPF 44Hz
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  switch (g) {
    case 0: cg = 131.0; break;
    case 1: cg = 65.5; break;
    case 2: cg = 32.8; break;
    case 3: cg = 16.4; break;
  }

  switch (a) {
    case 0: ca = 16384.0; break;
    case 1: ca = 8192.0; break;
    case 2: ca = 4096.0; break;
    case 3: ca = 2048.0; break;
  }

  // Calibração
  for (int i = 0; i < AMOSTRAGEM; i++) {

    imu_read();  

    bgx += ((float)dados[4] / cg);
    bgy += ((float)dados[5] / cg);
    bgz += ((float)dados[6] / cg);

    bax += ((float)dados[0] / ca);
    bay += ((float)dados[1] / ca);
    baz += ((float)dados[2] / ca);

    delayMicroseconds(1000); // 1000Hz
  }

  bgx /= AMOSTRAGEM;
  bgy /= AMOSTRAGEM;
  bgz /= AMOSTRAGEM;

  bax /= AMOSTRAGEM;
  bay /= AMOSTRAGEM;
  baz = (baz / AMOSTRAGEM) - 1.0;

  bias_reg = 1;

  return true;
}

bool imu_read() {
  static uint32_t last = micros();
  uint32_t now = micros();
  float dt = (now - last) * 1e-6f;
  last = now;

  // Configuração do passa-alta (frequência de corte ~0.05 Hz)
  float fc = 0.5f;                 // frequência de corte
  float tau = 1.0f / (2.0f * PI * fc);
  hp_alpha = tau / (tau + dt);

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14);

  if (Wire.available() < 14) return false;

  for (int i = 0; i < 7; i++) {
    uint8_t Hg = Wire.read();
    uint8_t Lw = Wire.read();
    dados[i] = int16_t((Hg << 8) | Lw);
  }

  if (bias_reg) {

    float gx = ((float)dados[4] / cg) - bgx;
    float gy = ((float)dados[5] / cg) - bgy;
    float gz = ((float)dados[6] / cg) - bgz;

    float ax = ((float)dados[0] / ca) - bax;
    float ay = ((float)dados[1] / ca) - bay;
    float az = ((float)dados[2] / ca) - baz;

    roll  += gx * dt;
    pitch += gy * dt;
    // Integra yaw normalmente
    yaw += gz * dt;
    // Aplica filtro passa-alta
    yaw_hp = hp_alpha * (yaw_hp + yaw - yaw_prev);
    yaw_prev = yaw;


    float roll_acc  = atan2(ay, az) * 180.0 / PI;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    roll  = ALPHA * roll  + (1.0f - ALPHA) * roll_acc;
    pitch = ALPHA * pitch + (1.0f - ALPHA) * pitch_acc;

    pgx = roll;
    pgy = pitch;
    pgz = yaw_hp;
}

  return true;
}

/* Getters */

float Gx() { return pgx; }
float Gy() { return pgy; }
float Gz() { return pgz; }

float VGx() { return ((float)dados[4] / cg) - bgx; }
float VGy() { return ((float)dados[5] / cg) - bgy; }
float VGz() { return ((float)dados[6] / cg) - bgz; }

float Ax() { return ((float)dados[0] / ca) - bax; }
float Ay() { return ((float)dados[1] / ca) - bay; }
float Az() { return ((float)dados[2] / ca) - baz; }