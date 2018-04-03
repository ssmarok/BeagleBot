#ifndef __IMU_H__
#define __IMU_H__

#define IMU_SAMPLE_RATE 100         // IMU Sample Rate (Hz)
#define IMU_MAGNETOMETER_STATUS 1   // Magnetometer On/Off
#define IMU_SHOW_WARNINGS 1       

void initializeIMU();
void imuInterruptFunc();
rc_imu_data_t* getIMUData();
int getIMUNewDataFlag();
void resetIMUNewDataFlag();

#endif
