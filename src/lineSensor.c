#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <math.h>
#include <stdint.h>
#include "lineSensor.h"
#include "driveTrain.h"

#define I2C_BUS 1
#define PORT_EXPANDER_ONE 0x3E
#define LINE_SENSOR_LEN 8

/*
Configures direction for each IO.
0 : IO is configured as an output
1 : IO is configured as an input
*/
#define RegDirA 0x0F
#define RegDirB 0x0E
#define FULL_MASK 0xFF

/*
Write: Data to be output to the output-configured IOs
Read: Data seen at the IOs, independent of the direction configured.
*/
#define RegDataA 0x10 // Back Sensor
#define RegDataB 0x11 // Front Sensor

/*
Enables high input mode for each [input-configured] IO
0 : OFF. VIH max = 3.6V and VCCx min = 1.2V
1 : ON. VIH max = 5.5V and VCCx min = 1.65V
*/
#define RegHighInputA 0x6A
#define RegHighInputB 0x69

/*
00 0x3E (0111110) First address of the 2-wire interface
01 0x3F (0111111) Second address of the 2-wire interface
10 0x70 (1110000) Third address of the 2-wire interface
11 0x71 (1110001) Fourth address of the 2-wire interface
*/

typedef int bool;
enum bool { false, true };
int frontSensor[LINE_SENSOR_LEN];
int backSensor[LINE_SENSOR_LEN];
int weightMap[LINE_SENSOR_LEN] = { 50, 15, 10, 0, 0, 10, 15, 50};

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
    rc_i2c_read_byte(I2C_BUS, RegDataB, &frontRegisterVal);
    printf("val:%d", frontRegisterVal);
    uint8_t backRegisterVal = 0;
    rc_i2c_read_byte(I2C_BUS, RegDataA, &backRegisterVal);
    if (frontRegisterVal >= 1) {
        for (int i=0; i<LINE_SENSOR_LEN; i++) {
            frontSensor[i] = (mask & frontRegisterVal) ? true: false;
            mask <<= 1;
        }
    }
    mask = 0x01;
    if (backRegisterVal >= 1) {
        for (int i=0; i<LINE_SENSOR_LEN; i++) {
            backSensor[i] = (mask & backRegisterVal) ? true: false;
            mask <<= 1;
        }
    }

    for (int i=0; i<LINE_SENSOR_LEN; i++) {
        printf("FrontBuffer[%d]: %d\n", i, frontSensor[i]);
    }
    for (int i=0; i<LINE_SENSOR_LEN; i++) {
        printf("BackBuffer[%d]: %d\n", i, backSensor[i]);
    }

    rc_i2c_release_bus(I2C_BUS);
}

int simpleLeftBiasBack(void) {
    int bias = 0;
    for (int i=0; i<4; i++) {
        bias += backSensor[i] ? weightMap[i] : 0;
    }
    return bias;
}

int simpleRightBiasBack(void) {
    int bias = 0;
    for (int i=4; i<LINE_SENSOR_LEN; i++) {
        bias += backSensor[i] ? weightMap[i] : 0;
    }
    return bias;
}

int simpleLeftBiasForward(void) {
    int bias = 0;
    for (int i=0; i<4; i++) {
        bias += frontSensor[i] ? weightMap[i] : 0;
    }
    return bias;
}

int simpleRightBiasForward(void) {
    int bias = 0;
    for (int i=4; i<LINE_SENSOR_LEN; i++) {
        bias += frontSensor[i] ? weightMap[i] : 0;
    }
    return bias;
}
