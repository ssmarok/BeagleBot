#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H

/* 
// DEFINED IN pinmap.h
#define FRONT_LEFT_LIMIT BLUE_SPI_PIN_6_SS2 
#define FRONT_RIGHT_LIMIT SPI_HEADER_PIN_5 
#define BACK_LEFT_LIMIT SPI_HEADER_PIN_4
#define BACK_RIGHT_LIMIT SPI_HEADER_PIN_3
*/

void initLimitSwitches(void);
int isSwitchClosed(int limitSwitch);
int isFrontCollision(void);
int isBackCollision(void);

#endif
