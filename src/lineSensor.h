#ifndef LINESENSOR_H
#define LINESENSOR_H

void initializeIRSensors();
void updateLineData();
int simpleLeftBiasBack(void);
int simpleLeftBiasForward(void);
int simpleRightBiasBack(void);
int simpleRightBiasForward(void);

#endif
