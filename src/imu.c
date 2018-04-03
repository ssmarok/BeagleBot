#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "imu.h"

static rc_imu_data_t imuData; 
static int imuNewDataFlag;

void initializeIMU(){
    rc_imu_config_t imuConf = rc_default_imu_config();
    imuConf.dmp_sample_rate = IMU_SAMPLE_RATE;
    imuConf.enable_magnetometer = IMU_MAGNETOMETER_STATUS;
    imuConf.show_warnings = IMU_SHOW_WARNINGS;

    if(rc_initialize_imu_dmp(&imuData, imuConf)){
        printf("IMU initialization FAILED!\n");
    } else {
        printf("IMU initialized\n");
    }

    rc_set_imu_interrupt_func(&imuInterruptFunc);
}

rc_imu_data_t* getIMUData(){
    return &imuData;
}

void imuInterruptFunc(){
    // TODO: Update position/rotation depending on time
    //printf("NEW IMU DATA\n");
    imuNewDataFlag = 1;
}

int getIMUNewDataFlag(){
    return imuNewDataFlag;
}

void resetIMUNewDataFlag(){
    imuNewDataFlag = 0;
}
