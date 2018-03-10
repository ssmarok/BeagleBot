#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "pinMap.h"
#include "terminus.h"
#include "driveTrain.h"
#include "lineSensor.h"
#include "shootingMechanism.h"
#include "encoders.h"

int SUBSTATE = 0;
float trigger= 0.0;

void initializeDriveTest(pthread_t pThread) {
	if(rc_initialize()){
		fprintf(stderr,"Initialization failed. Are you root?\n");
		exit(-1);
	}
    printDriveInstructions();
	pthread_create(&pThread, NULL, parseKeyboardInput, NULL);
	initializeDrivePins();
}

void initializeDriveThread(pthread_t pThread) {
	pthread_create(&pThread, NULL, runDriveThread, NULL);
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

void setSubState(int subState) {
    SUBSTATE = subState;
}

void *runDriveThread(void * param) {
    while (1) {
        switch (SUBSTATE) {
            case -1:
                break;
            case 0:
                drive(0, 0);
                break;
            case 1:
                //lineFollowForward();
                lineFollowForwardFast();
                break;
            case 2:
                lineFollowBackward();
                break;
            case 3:
                //turn(-90);
                resetEncoder(FRONT_LEFT);
                resetEncoder(FRONT_RIGHT);
                while (getEncoder(FRONT_LEFT) > -10500) {
                    drive(-70, 70);
                }
                SUBSTATE = 0;
                break;
            case 4:
                //turn(90);
                resetEncoder(FRONT_LEFT);
                resetEncoder(FRONT_RIGHT);
                while (getEncoder(FRONT_RIGHT) > -4500) {
                    drive(70, -70);
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

void turn(int degrees) {
    switch (degrees) {
        case -90:
            //resetEncoder(FRONT_RIGHT);
            while (getEncoder(FRONT_RIGHT) < 5000) {
                //drive(-100, 80);
                drive(-80, 80);
            }
            break;
        case 90:
            //resetEncoder(FRONT_RIGHT);
            while (getEncoder(FRONT_LEFT) > -5000) {
                drive(80, -80);
                //drive(80, -100);
            }
            break;
        default:
            break;
    }
    drive(0,0);
}

void *parseKeyboardInput(void * param){
	while (1) {
        char nextChar = getch();
        printf("---");
        switch (nextChar) {
            case 'r':
                resetEncoder(FRONT_LEFT);
                resetEncoder(FRONT_RIGHT);
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
                break;
            case 'q':
                drive(0, 0);
                printf("System Exit---\n\n");
                SUBSTATE = 0;
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
                setShootingMechanism();
                setShootingServo();
                break;
            case 'h':
                printf("HOLD");
                resetShootingMechanism();
                break;
            case 'e':
                printf("Front Right: %d\t", getEncoder(FRONT_RIGHT));
                printf("Front Left: %d", getEncoder(FRONT_LEFT));
                break;
            case 'l':
                printOutLineData();
                break;
            case 'c':
                printf("\033[0;0H");
                printf("\033[2J");
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

void turnRight90(void) {
    drive(60, 606);
}
void rotateCCW(void) {
    drive(-1*BASE_SPEED/2, BASE_SPEED/2);
}

void rotateCW(void) {
    drive(BASE_SPEED/2, -1*BASE_SPEED/2);
}

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

    lSpeed = lSpeed > 0 ? lSpeed : -1*lSpeed;
    rSpeed = rSpeed > 0 ? rSpeed : -1*rSpeed;

    rc_pwm_set_duty_mmap(0, 'A', (lSpeed/100.0) >= 1.0 ? MAX_DRIVE_SPEED : lSpeed/100.0);
    rc_pwm_set_duty_mmap(0, 'B', (rSpeed/100.0) >= 1.0 ? MAX_DRIVE_SPEED : rSpeed/100.0);
}

