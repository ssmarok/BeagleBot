#ifndef LINESENSOR_H
#define LINESENSOR_H

void initializeIRSensors();
void updateLineData();
int simpleLeftBias(int sensor[]);
int simpleRightBias(int sensor[]);

#endif
