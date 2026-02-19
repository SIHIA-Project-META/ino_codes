// Lógica (main code)
#include "imu.h"

#define IMU_F 1000
#define IMU_PERIOD_US (1000000 / IMU_F)

uint32_t last_time = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  imu_init(0,0); // 250°/s, 2g/s

  last_time = micros();   // inicializa o relógio
}

void loop() {

  uint32_t now = micros();

  if ((now - last_time) >= IMU_PERIOD_US) {

    last_time = now;

    imu_read();   // use filtro complementar

    Serial.print("Gx: ");
    Serial.print(Gx());
    Serial.print(", Gy: ");
    Serial.print(Gy());
    Serial.print(", Gz: ");
    Serial.println(Gz());
  }
}
