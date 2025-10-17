#include "MPU9250.h"

MPU9250 mpu;

void setup(){
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
  mpu.MPU_init();
  mpu.mag_init();

  mpu.gyro_calibrate();
  mpu.mag_calibrate();

}

void loop(){
  float dt = 0.001;
  float mx, my, mz, gx, gy, gz;

  mpu.mag_read(&mx, &my, &mz);
  mpu.gyro_read(&gx, &gy, &gz);

  float yaw = kalman_filter(gz, mx, my, 0.001, 0.002, 0.03);
  Serial.println(yaw);

}
