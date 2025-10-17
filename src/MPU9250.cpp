#include "MPU9250.h"

void read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest){
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(address, count);

  uint8_t i=0;
  while(Wire.available){
    dest[i++]=Wire.read();
  }

}

void write_bytes(uint8_t address, uint8_t subAddress, uint8_t data){
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission(true);
}

void MPU9250::MPU_init() {
  write_bytes(MPU9250_ADDR, MPU9250_PWR_MGMT1 , 0x00);
   //delay 100
   //optional check address of mpu using who am i register
  write_bytes(MPU9250_ADDR, MPU9250_PWR_MGMT1, 0x01); //clock speed
  //delay
  write_bytes(MPU9250_ADDR, MPU9250_GYRO_CONFIG, 0);
  //delay 
  write_bytes(MPU9250_ADDR, MPU9250_USER_CTRL, 0x20); //I2C master mode
  write_bytes(MPU9250_ADDR, MPU9250_I2C_MST_CTRL, 0x0D); // set clock speed
  write_bytes(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x02);
}

void MPU9250::mag_init(){
  write_bytes(AK8963_ADDR, AK8963_CNTL, 0x00);
  //delay
  write_bytes(AK8963_ADDR, AK8963_CNTL, 0x0F);///////make sure of this
  //delay
  uint8_t senstivity_data[3];
  read_bytes(AK8963_ADDR, AK8963_ASAX , 1, senstivity_data[0]);
  read_bytes(AK8963_ADDR, AK8963_ASAY, 1, senstivity_data[1]);
  read_bytes(AK8963_ADDR, AK8963_ASAZ, 1, senstivity_data[2]);
  
  magCallibration[0]=(float)(senstivity_data[0]-128)/256.0 + 1.0;
  magCallibration[1]=(float)(senstivity_data[1]-128)/256.0 + 1.0;
  magCallibration[2]=(float)(senstivity_data[2]-128)/256.0 + 1.0;

  write_bytes(AK8963_ADDR, AK8963_CNTL, 0x00);
  //delay
  //
  write_bytes(AK8963_ADDR, AK8963_CNTL, 0x16);
}
// Turns out i dont need acceleration but leaving it in just in case
void MPU9250::accel_calibrate(){
  double ax, ay, az;
  int num_samples = 100;
  for(int i=0;i<=num_samples;i++){
    accelOffsetX+=ax;
    accelOffsetY+=ay;
    accelOffsetZ+=az;
  }
  accelOffsetX/=num_samples;
  accelOffsetY/=num_samples;
  accelOffsetZ/=num_samples;
  // adjust gravity on axis -> depend on the placement of the sensor
  accelOffsetZ -=-1.0 // assume it is on the z axis
}

void MPU9250::gyro_calibrate(){
  double gx,gy,gz;
  int num_samples =100;
  for(int i=0;i<=num_samples;i++){
    gyroOffsetX+=gx;
    gyroOffsetY+=gy;
    gyroOffsetZ+=gz;

  }
  gyroOffsetX/=num_samples;
  gyroOffsetY/=num_samples;
  gyroOffsetZ/=num_samples;
}

void MPU9250::mag_calibrate(){ ///////////////////revise
  float magMIN[3]={10000, 10000,10000};
  float magMAX[3]={-10000,-10000,-10000};
  unsigned long startTime=millis();
  while(millis()-startTime<10000){
    mag_read()
    if(mx<magMIN[0]) magMIN[0]=mx;
    if(mx<magMIN[1]) magMIN[1]=my;
    if(mx<magMIN[2]) magMIN[2]=mz;
    
    if(mx<magMAX[0]) magMIN[0]=mx;
    if(mx<magMAX[1]) magMIN[1]=my;
    if(mx<magMAX[2]) magMIN[2]=mz;
  }
  //delay
} 

void MPU9250::gyro_read(float* gx, float* gy, float* gz){
  uint8_t rawData[6];
  read_bytes(MPU9250_ADDR, MPU9250_GYRO_XOUT_H, 6, rawData);
  int16_t gyroX = ((int16_t)rawData[0]<<8) | rawData[1];
  int16_t gyroY = ((int16_t)rawData[2]<<8) | rawData[3];
  int16_t gyroZ = ((int16_t)rawData[4]<<8) | rawData[5];
  *gx=(float) gyroX/131.0-gyroOffsetX;
  *gy=(float) gyroY/131.0-gyroOffsetY;
  *gz = (float) gyroZ/131.0-gyroOffsetZ;
}
void MPU9250::mag_read(float* mx, float* my, float* mz){
  uint8_t rawData[7];
  read_bytes(AK8963_ADDR, AK8963_MX_L ,7, rawData);
  int16_t magX = ((int16_t)rawData[1]<<8) | rawData[0];
  int16_t magY = ((int16_t)rawData[3]<<8) | rawData[2];
  int16_t magZ = ((int16_t)rawData[5]<<8) | rawData[4];

  *mx=(magX - (magMIN[0]+magMAX[0])/2) * MAG_SCALE;
  *my=(magY-(magMIN[1]-magMAX[1])/2) * MAG_SCALE;
  *mz=(magZ-(magMIN[2]-magMAX[2]/2) * MAG_SCALE;
}

double MPU9250::getGyro_Yaw(float gz, float dt){
  long prev_time = millis();
  float yaw = 0;
  yaw +=gx*dt;
  return yaw;
}
double MPU9250::getMag_Yaw(float mx, float my){
  return atan(my, mx)*180.0/PI;
}

double kalman_filter(float gz, float mx, float my, float Q_angle = 0.0, float Q_bias = 0.0, float R_measure = 0.0){
  float x[2] = {0.0, 0.0}; //state vector {angle, bias}
  float P[4] = {1.0, 0.0, 0.0, 1.0}; // covariance matrix bbin/
  static bool init = false;

  float magYaw = atan2(my, mx);
  //optional convert to 360 notation if(magYaw<0) magYaw+=360.0;
  if(!init){
    x[0] = magYaw;
    x[1]=0.0;
    init = true;
  }
  // This is to get reading for first time dont really understand this part but guy on github did it like this
  float rate = gz-x[1]; // calculate bias rate
  x[0] += dt*rate; // update angle to angle from gyro integration
  float Q[4] = {Q_angle, 0.0,0.0,Q_bias}; // process noise covariance matrix

  float P_temp[4]; // begin covariance prediction P= F*P*F'+Q
  P_temp[0] = P[0] + dt * (dt * P[3] - P[1]-P[2] + Q_angle);
  P_temp[1] = P[1] - dt * P[3];
  P_temp[2] = P[2] - dt * P[3];
  P_temp[3] = P[3] + Q_bias * dt;

  dsps_memcpy_f32(P,P_temp, 4); // update the main covariance preiction matrix
  //optional convert the angle into 360 notation
  //do later too tired now
  
  float z = magYaw;  // measurement
  float diff = z - x[0];
  if (diff > 180.0) z -= 360.0;
  else if (diff < -180.0) z += 360.0;
  
  float S = P[0] + R_measure; // calculate innovation covariance matrix

  float S_inv = 1.0 / S;
  
  float K[2]; // begin calculating kalman gain
  K[0] = P[0] * S_inv;  // P[0][0] / S
  K[1] = P[2] * S_inv;  // P[1][0] / S
  
  float y = z - x[0];  // measurement - predition
  
  dsps_mulc_f32(K, y, K, 2, 1, 1);  // update states
  dsps_add_f32(x, K, x, 2, 1, 1); 
  
  //TODO: understand what the hell happened from here  
  float IKH[4] = {1.0 - K[0], 0.0, -K[1], 1.0};
  
  float P_new[4];
  dsps_mat_mul_f32(IKH, P, P_new, 2, 2, 2);
  dsps_memcpy_f32(P, P_new, 4);
  
  float output = x[0];
  while (output >= 360.0) output -= 360.0;
  while (output < 0) output += 360.0;
  
  return output;
}


