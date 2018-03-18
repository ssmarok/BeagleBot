#ifndef __RUNMODE__
#define __RUNMODE__

// Operational mode
typedef int RUN_MODE;
enum RUN_MODE { MANUAL=0, AUTONOMOUS=1};


int getRunMode();
void toggleRunMode();
void setRunMode();

#endif
