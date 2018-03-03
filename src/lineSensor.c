#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <math.h>
#include <stdint.h>
#include "lineSensor.h"
#include "driveTrain.h"

typedef int bool;
enum bool { false, true };
int offPath = 0;
int frontSensor[LINE_SENSOR_LEN];
int backSensor[LINE_SENSOR_LEN];

int leftWeightMap[LINE_SENSOR_LEN] = { 60, 15, 10, 0, 0, -5, -10, -60};
int rightWeightMap[LINE_SENSOR_LEN] = { -60, -10, -5, 0, 0, 10, 15, 60};

int prevLBias = 0;
int prevRBias = 0;

void initializeIRSensors() {
    rc_i2c_init(I2C_BUS, PORT_EXPANDER_ONE);
    rc_i2c_write_byte(I2C_BUS, RegDirA, FULL_MASK);
    rc_i2c_write_byte(I2C_BUS, RegDirB, FULL_MASK);
    rc_i2c_write_byte(I2C_BUS, RegHighInputA, FULL_MASK);
    rc_i2c_write_byte(I2C_BUS, RegHighInputB, FULL_MASK);
    printf("Initialization of IR Sensors Complete\n");
}

void updateLineData() {
    rc_i2c_claim_bus(I2C_BUS);
    int mask = 0x01;

    uint8_t frontRegisterVal = 0;
    uint8_t backRegisterVal = 0;
    rc_i2c_read_byte(I2C_BUS, RegDataB, &frontRegisterVal);
    rc_i2c_read_byte(I2C_BUS, RegDataA, &backRegisterVal);
    if (frontRegisterVal > 0) {
        for (int i=0; i<LINE_SENSOR_LEN; i++) {
            frontSensor[LINE_SENSOR_LEN-i-1] = ((mask & frontRegisterVal)>0) ? true: false;
            mask <<= 1;
        }
    }
    if (backRegisterVal > 0) {
        mask = 0x01;
        for (int i=0; i<LINE_SENSOR_LEN; i++) {
            backSensor[i] = (mask & backRegisterVal) ? true: false;
            if (backSensor[i]) {
               // printf("Cool");
            }
            else {
               // printf("Boo");
            }
            mask <<= 1;
        }
    }

    rc_i2c_release_bus(I2C_BUS);
    usleep(1000);
}

int sensorCount(int sensor[]) {
    int count = 0;
    for (int i=0; i<LINE_SENSOR_LEN; i++) {
        count += sensor[i] ? 1 : 0;
    }
    return count;
}

int frontSensorCount(void) {
    int count = 0;
    for (int i=0; i<LINE_SENSOR_LEN; i++) {
        count += frontSensor[i] ? 1 : 0;
    }
    return count;
}

int centerBias(void) {
    return ((frontSensorCount() == 2 || frontSensorCount() == 3) && frontSensor[3]
    && frontSensor[4]) ? 1000: 0;
}

int isCentered(void) {
    return (frontSensorCount() == 2 && frontSensor[3] & frontSensor[4]);
}

int isFullLine(void) {
    return (frontSensorCount() > 6);
}

int calculateBias(int sensor[], int weightMap[]) {
    int bias = 0;
    for (int i=0; i<LINE_SENSOR_LEN; i++) {
        bias += sensor[i] ? weightMap[i] : 0;
    }
    if (frontSensorCount() > 1) {
        return bias;
    }
    return prevLBias;
}

int rightBias(void) {
    int bias = 0;
    for (int i=0; i<LINE_SENSOR_LEN; i++) {
        bias += frontSensor[i] ? rightWeightMap[i] : 0;
    }
    if (frontSensorCount() > 1) {
        return bias;
    }
    return prevRBias;
}

void lineFollowForward(void) {
    updateLineData();
    int lBias = calculateBias(frontSensor, leftWeightMap);
    int rBias = calculateBias(frontSensor, rightWeightMap);

    drive(BASE_SPEED-lBias, BASE_SPEED-rBias);
    prevLBias = lBias;
    prevRBias = rBias;
}

void lineFollowBackward(void) {
    updateLineData();
    // Sensor is in reverse
    int lBias = calculateBias(backSensor, rightWeightMap);
    int rBias = calculateBias(backSensor, leftWeightMap);

    drive(-1*(BASE_SPEED-lBias), -1*(BASE_SPEED-rBias));
    prevLBias = lBias;
    prevRBias = rBias;
}
