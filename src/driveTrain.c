#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "driveTrain.h"
#include "encoders.h"
#include "fsm.h"
#include "limitSwitch.h"
#include "lineSensor.h"
#include "odometry.h"
#include "pinMap.h"
#include "runMode.h"
#include "shootingMechanism.h"
#include "terminus.h"
//#include "sound.h"

/* DRIVESTATE used in fsm.c. Commendted here for reference */
// enum DRIVESTATE { NOP, DRIVE_STOP, FOLLOW_FORWARD, FOLLOW_BACKWARD, TURN_NEG_90, TURN_POS_90 };
int SUBSTATE = 0;

void initializeDriveThread(pthread_t pThread) {
	pthread_create(&pThread, NULL, runDriveThread, NULL);
}

void initializeKeyboardThread(pthread_t pThread) {
	pthread_create(&pThread, NULL, parseKeyboardInput, NULL);
}

void initializeDrivePins(){
    int driveMotorPins[] = { MOTOR_A_0, MOTOR_A_1, MOTOR_FIRE};
    for (int i=0; i < 4; i++) {
        rc_gpio_export(driveMotorPins[i]);
        rc_gpio_set_dir(driveMotorPins[i], OUTPUT_PIN);
    }
    // Pinmux mode to PWM
    rc_set_pinmux_mode(PWM_0A, PINMUX_PWM);
    rc_set_pinmux_mode(PWM_0B, PINMUX_PWM);
    // Initialize PWM subsystem
    rc_pwm_init(0, PWM_FREQUENCY);
    //Initialize Servo Pins
    rc_enable_servo_power_rail();
    //Initialize Line Sensors
    initializeIRSensors();
}

/* DRIVESTATE used in fsm.c. Commendted here for reference */
// enum DRIVESTATE { NOP, DRIVE_STOP, FOLLOW_FORWARD, FOLLOW_BACKWARD, TURN_NEG_90, TURN_POS_90 };
void setSubState(int subState) {
    /*
    static const char *subStateNames[] = {
        "NOP", "DRIVE_STOP", "FOLLOW_FORWARD", "FOLLOW_BACKWARD", "TURN_NEG_90", "TURN_POS_90"
    }; */
    //printf("DRIVESTATE: %s SUBSTATE: %d \n", subStateNames[subState+1], subState);  // +1 for -1 case
    SUBSTATE = subState;
}

void *runDriveThread(void * param) {
    double start_orientation = 0; // Can't be declared in case statement
    while (1) {
        switch (SUBSTATE) {
            case -1:    // DRIVESTATE: NOP
                break;
            case 0:     // DRIVESTATE: DRIVE_STOP     
                drive(0, 0);
                break;
            case 1:     // DRIVESTATE: FOLLOW_FORWARD
                //lineFollowForward();
                lineFollowForwardFast();
                break;
            case 2:     // DRIVESTATE: FOLLOW_BACKWARD
                lineFollowBackward();
                break;
            case 3:     // DRIVESTATE: TURN_NEG_90 (LEFT)
                //turn(-90);
                
                /*
                resetEncoder(FRONT_LEFT_ENCODER);
                resetEncoder(FRONT_RIGHT_ENCODER);
                while (getEncoder(FRONT_LEFT_ENCODER) > -10500) {
                    drive(-70, 70);
                } */

                start_orientation = getOrientation();
                while(getOrientation() < start_orientation + 1.56) { // 90 degrees in radians
                    drive(-70, 70);
                }
                SUBSTATE = 0;
                break;
            case 4:     // DRIVESTATE: TURN_POS_90 (RIGHT)
                //turn(90);

                /*
                resetEncoder(FRONT_LEFT_ENCODER);
                resetEncoder(FRONT_RIGHT_ENCODER);
                printf("First encoder value: %d\n", getLastEncoder(FRONT_LEFT_ENCODER));
                while (getEncoder(FRONT_LEFT_ENCODER) < 8500) {
                    //printf("turning...\n");
                    //printf("Last encoder value: %d\n", getLastEncoder(FRONT_LEFT_ENCODER));
                    drive(90, -90);
                    //usleep(100);
                }
                printf("Leaving turn 90 deg right drivestate\n");
                printf("Last encoder value: %d\n", getLastEncoder(FRONT_LEFT_ENCODER));
                */

                start_orientation = getOrientation();
                while(getOrientation() > start_orientation - 1.56) { // 90 degrees in radians
                    drive(70, -80);
                }
                SUBSTATE = 0;
                break;
            default:
                usleep(2000);
                break;
        }
	usleep(50);
    }
	return 0; // Exits void thread
}

/*
void turn(int degrees) {
    switch (degrees) {
        case -90:
            //resetEncoder(FRONT_RIGHT_ENCODER);
            while (getEncoder(FRONT_RIGHT_ENCODER) < 5000) {
                //drive(-100, 80);
                drive(-80, 80);
            }
            break;
        case 90:
            //resetEncoder(FRONT_RIGHT_ENCODER);
            while (getEncoder(FRONT_LEFT_ENCODER) > -5000) {
                drive(80, -80);
                //drive(80, -100);
            }
            break;
        default:
            break;
    }
    drive(0,0);
}
*/

void *parseKeyboardInput(void * param){
	while (1) {
        char nextChar = getch();
        printf("---");
        switch (nextChar) {
            case 'r':
                resetEncoder(FRONT_LEFT_ENCODER);
                resetEncoder(FRONT_RIGHT_ENCODER);
                break;
            case 'w':
                printf("Forward");
                SUBSTATE = 1;
                break;
            case 'W':
                drive(BASE_SPEED, BASE_SPEED);
                SUBSTATE = -1;
                break;
            case 's':
                printf("Reverse");
                SUBSTATE = 2;
                break;
            case 'S':
                drive(-1*BASE_SPEED, -1*BASE_SPEED);
                SUBSTATE = -1;
                break;
            case 'd':
                printf("Right Turn");
                SUBSTATE = 4;
                break;
            case 'a':
                printf("Left Turn");
                SUBSTATE = 3;
                break;
            case 'D':
                drive(MAX_SPEED, -1*MAX_SPEED);
                SUBSTATE = -1;
                break;
            case 'A':
                drive(-1*MAX_SPEED, MAX_SPEED);
                SUBSTATE = -1;
                break;
            case 'P':
                drive(0, 0);
                printf("Halt");
                SUBSTATE = 0;
                setRunMode(MANUAL);
                resetFSM();
                break;
            case 'q':
                drive(0, 0);
                printf("System Exit---\n\n");
                SUBSTATE = 0;
	            rc_set_state(EXITING); 
                rc_cleanup();
                exit(0);
            case 'y':
                rc_send_servo_pulse_normalized_all(-1.0);
                usleep(10000);
                break;
            case 't':
                rc_send_servo_pulse_normalized_all(1.0);
                usleep(10000);
                break;
            case 'f':
                printf("FIRE");
                /*
                playSound();
                usleep(1000000);
                usleep(1000000);
                usleep(1000000);
                */
                setShootingMechanism();
                usleep(1000000);
                setShootingServo();
                break;
            case 'h':
                printf("HOLD");
                resetShootingMechanism();
                break;
            case 'e':
                printf("Front Left: %d\t", getEncoder(FRONT_LEFT_ENCODER));
                printf("Front Right: %d\t", getEncoder(FRONT_RIGHT_ENCODER));
                break;
            case 'l':
                printOutLineData();
                break;
            case 'i':
                printOutLimitSwitchData();
                break;
            case 'o':
                setOdomPrintDataFlag();
                break;
            case 'x':
                printFSMState();
                break;
            case 'c':
                printf("\033[0;0H");
                printf("\033[2J");
                break;
            case '1':
                setRunMode(MANUAL);
                printf("Operational Mode: Manual");
                break;
            case '2':
                setRunMode(AUTONOMOUS);
                printf("Operational Mode: Autonomous");
                break;
            case '?':
                printDriveInstructions();
                break;
            default:
                printf("Running Thread");
                break;
        }
        printf("\n");
	usleep(5000);
	}
	return 0; // Exits void thread
}

void driveForward(void) {
    drive(BASE_SPEED, BASE_SPEED);
}

void driveBackward(void) {
    drive(-1*BASE_SPEED, -1*BASE_SPEED);
}
/*
void turnRight90(void) {
    drive(60, 606);  // Make right speed negative
}
*/
/*
void rotateCCW(void) {
    drive(-1*BASE_SPEED/2, BASE_SPEED/2);
}

void rotateCW(void) {
    drive(BASE_SPEED/2, -1*BASE_SPEED/2);
}
*/

// Sets minimum speed
int bufferSpeed(int speed, int buffer) {
    if (speed == 0) {
        return speed;
    }
    if (speed < 0) {
        if (speed > -1*buffer) {
            return -1*buffer;
        }
    }
    else {
        if (speed < buffer) {
            return buffer;
        }
    }
    return speed;
}

void drive(int lSpeed, int rSpeed) {
    lSpeed = bufferSpeed(lSpeed, 20);
    rSpeed = bufferSpeed(rSpeed, 20);
    rc_gpio_set_value_mmap(MOTOR_A_0, lSpeed >= 0 ? LOW: HIGH);
    rc_gpio_set_value_mmap(MOTOR_A_1, rSpeed >= 0 ? HIGH: LOW);


    // Make negative speed positive for use with PWM function
    lSpeed = lSpeed > 0 ? lSpeed : -1*lSpeed;
    rSpeed = rSpeed > 0 ? rSpeed : -1*rSpeed;

    rc_pwm_set_duty_mmap(0, 'A', (lSpeed/100.0) >= 1.0 ? MAX_DRIVE_SPEED : lSpeed/100.0);
    rc_pwm_set_duty_mmap(0, 'B', (rSpeed/100.0) >= 1.0 ? MAX_DRIVE_SPEED : rSpeed/100.0);
}

