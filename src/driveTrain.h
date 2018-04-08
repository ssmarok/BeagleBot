/** @file driveTrain.h
 *  @brief Function prototypes for the overall drive system. 
 *
 *  This contains the prototypes for the main drivetrain 
 *  and eventually any macros, constants, or global
 *  variables necessary. 
 *
 *  @bug No known bugs.
 */

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

/* Definitions */
#define BASE_SPEED 50
#define MAX_SPEED 100
#define MAX_DRIVE_SPEED 0.99

/* Function declarations */
void initializeDriveThread(pthread_t pThread);
void initializeKeyboardThread(pthread_t pThread);
void initializeDrivePins();

void setSubState(int subState);
void *runDriveThread(void * param);
void *parseKeyboardInput(void * param);

void turnLeft90();
void turnRight90();
int bufferSpeed(int speed, int buffer);
void drive(int lSpeed, int rSpeed);

#endif
