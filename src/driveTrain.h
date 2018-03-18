#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#define BASE_SPEED 50
#define MAX_SPEED 100
// Definitions
#define MAX_DRIVE_SPEED 0.99
/* 
// DEFINED in pinmap.h
#define MOTOR_A_0 BLUE_GP0_PIN_3
#define MOTOR_A_1 BLUE_GP0_PIN_4
#define PWM_0A GPS_HEADER_PIN_3
#define PWM_0B GPS_HEADER_PIN_4
#define PWM_FREQUENCY 20000
*/

void initializeDriveThread(pthread_t pThread);
void initializeDrivePins();
void initializeKeyboardThread(pthread_t pThread);
void *parseKeyboardInput(void * param);
void *runDriveThread(void * param);
void drive(int lSpeed, int rSpeed);
void releaseTrigger();
void holdTrigger();
void releaseServo();
void holdServo();

void setSubState(int subState);
void rotateCW(void);
void rotateCCW(void);
void turn(int degrees);

void turnRight90(void);
void setBaseSpeed(int baseSpeed);

#endif
