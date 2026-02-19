#ifndef IMU_H
#define IMU_H

bool imu_init(int, int);
bool imu_read();

float Ax ();
float Ay ();
float Az ();
float VGx ();
float VGy ();
float VGz ();

float Gx ();
float Gy ();
float Gz ();

#endif
