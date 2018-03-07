#ifndef LINESENSOR_H
#define LINESENSOR_H

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
10 0x70 (1110000) Third address of the2-wire interface
11 0x71 (1110001) Fourth address of the 2-wire interface
*/

void initializeIRSensors();
void updateLineData();
int frontSensorCount(void);
int isCentered(int sensor[]);
int isFullLineFront(void);
int isFullLineBack(void);

void lineFollowForward(void);
void lineFollowForwardFast(void);
void lineFollowBackward(void);
#endif
