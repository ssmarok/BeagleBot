#ifndef SHOOTINGMECHANISM_H
#define SHOOTINGMECHANISM_H

#define TUBE_SERVO_CHANNEL 1
#define BUCKET_SERVO_CHANNEL 3
#define SERVO_SLEEP_TIME 100000     // In Microseconds
#define SHOOT_ROUTINE_TIME 5000000  // In microseconds

void releaseTrigger();
void holdTrigger();
void releaseServo();
void holdServo();

void initializeServoThread(pthread_t pThread);
void *runServoThread(void * param);
void setShootingServo();
void setShootingMechanism();
void resetShootingMechanism();

void releaseBucket(int direction);

#endif
