#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <math.h>
#include <stdint.h>
#include "lineSensor.h"
#include "driveTrain.h"
#include "./../api/libraries/bb_blue_api.h"

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
#define RegDataA 0x10
#define RegDataB 0x11

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

void initializeIRSensors() {
    rc_i2c_init(I2C_BUS, PORT_EXPANDER_ONE);
    rc_i2c_write_byte(I2C_BUS, RegDirA, FULL_MASK);
    rc_i2c_write_byte(I2C_BUS, RegDirB, FULL_MASK);
    rc_i2c_write_byte(I2C_BUS, RegHighInputA, FULL_MASK);
    rc_i2c_write_byte(I2C_BUS, RegHighInputB, FULL_MASK);
}

void updateLineData() {
    rc_i2c_claim_bus(I2C_BUS);

    int mask = 0x01;
    uint8_t buffer[128];
    uint8_t frontRegisterVal = rc_i2c_read_byte(I2C_BUS, RegDataA, buffer);
    printf("Front Sensor Integer: %d\n", frontRegisterVal);
    printf("Back Sensor Hex: %02x\n", frontRegisterVal);
    uint8_t backRegisterVal = rc_i2c_read_byte(I2C_BUS, RegDataA, buffer);
    printf("Back Sensor Integer: %d\n", backRegisterVal);
    printf("Back Sensor Hex: %02x\n", backRegisterVal);
    for (int i=0; i<LINE_SENSOR_LEN; i++) {
        frontSensor[i] = (mask & frontRegisterVal) ? true: false;
        mask <<= 1;
    }
    mask = 0x01;
    for (int i=0; i<LINE_SENSOR_LEN; i++) {
        backSensor[i] = (mask & backRegisterVal) ? true: false;
        mask <<= 1;
    }

    rc_i2c_release_bus(I2C_BUS);
}

/*
int simpleLeftBias(int sensor[]) {
    int bias = 0;
    for (int i=0; i<4; i++) {
        bias += sensor[i] ? weightMap[i] : 0;
    }
    return bias;
}

int simpleRightBias(int sensor[]) {
    int bias = 0;
    for (int i=4; i<LINE_SENSOR_LEN; i++) {
        bias += sensor[i] ? weightMap[i] : 0;
    }
    return bias;
}
*/
