#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

// Definitions
#define MAX_DRIVE_SPEED 100
#define MOTOR_A_0 BLUE_GP0_PIN_3
#define MOTOR_A_1 BLUE_GP0_PIN_4
#define MOTOR_B_0 BLUE_GP0_PIN_5
#define MOTOR_B_1 BLUE_GP0_PIN_6
#define PWM_0A GPS_HEADER_PIN_3
#define PWM_0B GPS_HEADER_PIN_4
#define PWM_FREQUENCY 20000

void initializeDriveTest(pthread_t pThread);
void initializeDrivePins();
void *parseKeyboardInput(void * param);
void drive(int lSpeed, int rSpeed);

#endif
