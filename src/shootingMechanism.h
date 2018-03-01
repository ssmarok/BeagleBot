#ifndef SHOOTINGMECHANISM_H
#define SHOOTINGMECHANISM_H

void releaseTrigger();
void holdTrigger();
void releaseServo();
void holdServo();

void initializeServoThread(pthread_t pThread);
void *runServoThread(void * param);
void setShootingMechanism();
void resetShootingMechanism();

#endif
