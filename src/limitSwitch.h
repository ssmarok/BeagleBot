#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H

/* 
// DEFINED IN pinmap.h
#define FRONT_LEFT_LIMIT BLUE_SPI_PIN_6_SS2 
#define FRONT_RIGHT_LIMIT SPI_HEADER_PIN_5 
#define BACK_LEFT_LIMIT SPI_HEADER_PIN_4
#define BACK_RIGHT_LIMIT SPI_HEADER_PIN_3
*/
//const char *limitSwitchArrayNames[4] = {"FRONT_LEFT_LIMIT", "FRONT_RIGHT_LIMIT", "BACK_LEFT_LIMIT", "BACK_RIGHT_LIMIT"};

void initLimitSwitches(void);
int isSwitchClosed(int limitSwitch);
int isFrontCollision(void);
int isBackCollision(void);
void printOutLimitSwitchData(void);

#endif
