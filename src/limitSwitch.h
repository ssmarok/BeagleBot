#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H

void initLimitSwitches(void);
int isSwitchClosed(int limitSwitch);
int isFrontCollision(void);
int isBackCollision(void);

#endif
