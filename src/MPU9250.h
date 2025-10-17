#ifndef MPU9250_H 
#define MPU9250_H

#include <Wire.h>
#include <cstdint>
#include "esp_dsp.h"
#include <math.h>
#include<freertos/FreeRTOS.h>

#define MPU9250_ADDR 0x68
#define AK8963_ADDR 0x0C

#define MPU9250_CONFIG 0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_I2C_MST_CTRL 0x24
#define MPU9250_USER_CTRL 0x6A
#define MPU9250_PWR_MGMT1 0x6C
#define MPU9250_INT_PIN_CFG 0x37

#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_XOUT_L 0x3C
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_YOUT_L 0x3E
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_ACCEL_ZOUT_L 0x40
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_XOUT_L 0x44
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_YOUT_L 0x46
#define MPU9250_GYRO_ZOUT_H 0x47
#define MPU9250_GYRO_ZOUT_L 0x48

#define AK8963_CNTL 0x0A
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

#define AK8963_MX_L 0x03
#define AK8963_MX_H 0x04
#define AK8963_MY_L 0X05
#define AK8963_MY_H 0x06
#define AK8963_MZ_L 0x07
#define AK8963_MZ_H 0x08

#define MAG_SCALE 0.15 // TODO: change later
#define GYRO_SCALE 131.0
class MPU9250 {
  public:
      float magCallibration[3];
      void MPU_init();
      void mag_init();

      void accel_calibrate();
      void gyro_calibrate();
      void mag_calibrate(); 

      void accel_read(float *ax, float *ay, float *az);
      void gyro_read(float *gx, float *gy, float *gz);
      void mag_read(float *mx, float *my, float *mz);

      double getPitchRoll();
      double getGyro_Yaw(float gz, float dt);
      double getMag_Yaw(float mx, float my);
};

double kalman_filter(float gz, float mx, float my, float dt, float Q_angle=0.0, float Q_bias = 0.0, float R_measure = 0.0);

void read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);
void write_bytes(uint8_t address, uint8_t subAddress, uint8_t data);


#endif // !

