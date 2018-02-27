#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

// Definitions
#define MAX_DRIVE_SPEED 0.99
#define MOTOR_A_0 BLUE_GP0_PIN_3
#define MOTOR_A_1 BLUE_GP0_PIN_4
//#define MOTOR_B_0 BLUE_GP0_PIN_5
//#define MOTOR_B_1 BLUE_GP0_PIN_6
//#define MOTOR_TRIGGER BLUE_GP1_PIN_3
//#define MOTOR_TRIGGER_2 BLUE_GP1_PIN_4
#define PWM_0A GPS_HEADER_PIN_3
#define PWM_0B GPS_HEADER_PIN_4
#define PWM_FREQUENCY 20000
#define MOTOR_FIRE BLUE_GP0_PIN_5

void initializeDriveTest(pthread_t pThread);
void initializeSubState(pthread_t pThread);
void initializeDrivePins();
void *parseKeyboardInput(void * param);
void *trySubstate(void * param);
void drive(int lSpeed, int rSpeed);
void releaseTrigger();
void holdTrigger();
void releaseServo();
void holdServo();

void lineFollowForward(void);
void lineFollowBackward(void);
void rotateCW(void);
void rotateCCW(void);

void initializeShootState(pthread_t pThread);
void *tryShootState(void * param);
#endif
