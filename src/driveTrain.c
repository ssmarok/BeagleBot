#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "terminus.h"
#include "driveTrain.h"
#include "lineSensor.h"

int left = 50;
int right = 50;
int SUBSTATE = 0;
float trigger= 0.0;
int shootState = 0;

#define BASE_SPEED 50

void initializeDriveTest(pthread_t pThread) {
	if(rc_initialize()){
		fprintf(stderr,"Initialization failed. Are you root?\n");
		exit(-1);
	}
    printDriveInstructions();
	pthread_create(&pThread, NULL, parseKeyboardInput, NULL);
	initializeDrivePins();
}

void initializeSubState(pthread_t pThread) {
	pthread_create(&pThread, NULL, trySubstate, NULL);
}

void initializeShootState(pthread_t pThread) {
	pthread_create(&pThread, NULL, tryShootState, NULL);
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

void *trySubstate(void * param) {
    while (1) {
        switch (SUBSTATE) {
            case 0:
                //drive(0, 0);
                usleep(2000);
                break;
            case 1:
                //lineFollowForward();
                drive(left, right);
                break;
            case 2:
                lineFollowBackward();
                break;
            case 3:
                usleep(2000);
                rotateCCW();
                break;
            case 4:
                usleep(2000);
                rotateCW();
                break;
            default:
                usleep(2000);
                break;
        }
	usleep(10000);
    }
	return 0; // Exits void thread
}

void *parseKeyboardInput(void * param){
	while (1) {
        char nextChar = getch();
        printf("---");
        switch (nextChar) {
            case 'w':
                drive(left, right);
                printf("Forward");
                //lineFollowForward();
                //SUBSTATE = 1;
                break;
            case 's':
                //drive(-1*left, -1*right);
                printf("Reverse");
                //lineFollowBackward();
                SUBSTATE = 2;
                break;
            case 'd':
                drive(left, -1*right);
                printf("Right Turn");
                SUBSTATE = 4;
                break;
            case 'a':
                drive(-1*left, right);
                printf("Left Turn");
                SUBSTATE = 3;
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
            case 'o':
                left = (left < 100 ? left+1 : 100);
                //trigger = (trigger < 1.0 ? trigger +0.5 : 1.0);
                //printf("Increase left: %d", left);
                printf("Increase trigger: %f", trigger);
                break;
            case 'p':
                right = (right < 100 ? right+1 : 100);
                printf("Increase right: %d", right);
                break;
            case 'k':
                left = (left > 0 ? left-1 : 0);
                //trigger = (trigger > -1.0 ? trigger-0.5 : -1.0);
                //printf("Decrease left: %d", left);
                printf("Decrease trigger: %f", trigger);
                break;
            case 'l':
                right = (right > 0 ? right-1 : 0);
                printf("Decrease right: %d", right);
                break;
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
		shootState = 1;
                //releaseTrigger();
                //releaseServo();
                break;
            case 'h':
                printf("HOLD");
		shootState = 0;
                //holdTrigger();
                //holdServo();
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

void *tryShootState(void * param) {
    while(1) {
	switch(shootState){
	    case 0:
		holdTrigger();
		holdServo();
		break;
	    case 1:
		releaseTrigger();
		releaseServo();
		break;
	}
	usleep(10000);
    }
    return 0;
}

void lineFollowForward(void) {
    updateLineData();
    drive(BASE_SPEED+simpleLeftBiasForward(), BASE_SPEED+simpleRightBiasForward());
}

void lineFollowBackward(void) {
    updateLineData();
    drive(-1*BASE_SPEED-simpleLeftBiasBack(), -1*BASE_SPEED-simpleRightBiasBack());
}

void rotateCCW(void) {
    drive(-1*BASE_SPEED, BASE_SPEED);
}

void rotateCW(void) {
    drive(BASE_SPEED, -1*BASE_SPEED);
}

void drive(int lSpeed, int rSpeed) {
    rc_gpio_set_value_mmap(MOTOR_A_0, lSpeed >= 0 ? LOW: HIGH);
    rc_gpio_set_value_mmap(MOTOR_A_1, rSpeed >= 0 ? HIGH: LOW);

    lSpeed = lSpeed > 0 ? lSpeed : -1*lSpeed;
    rSpeed = rSpeed > 0 ? rSpeed : -1*rSpeed;

    rc_pwm_set_duty_mmap(0, 'A', (lSpeed/100.0) >= 1.0 ? MAX_DRIVE_SPEED : lSpeed/100.0);
    rc_pwm_set_duty_mmap(0, 'B', (rSpeed/100.0) >= 1.0 ? MAX_DRIVE_SPEED : rSpeed/100.0);
}

void releaseTrigger() {
    rc_gpio_set_value_mmap(MOTOR_FIRE, HIGH);
    //usleep(10000);
    //rc_send_servo_pulse_normalized_all(-1.5);
    //rc_send_servo_pulse_us_all(1500);
/*
    rc_gpio_set_value_mmap(MOTOR_TRIGGER, HIGH);
    rc_gpio_set_value_mmap(MOTOR_TRIGGER_2, HIGH);
    rc_set_led(GREEN, HIGH);
*/
}

void holdTrigger() {
    rc_gpio_set_value_mmap(MOTOR_FIRE, LOW);
    //usleep(10000);
    //rc_send_servo_pulse_normalized_all(-0.5);
    //rc_send_servo_pulse_us_all(2500);
    //rc_set_led(GREEN, LOW);
}

void releaseServo(){
    rc_send_servo_pulse_normalized(1, -1);
    //usleep(22000);
}

void holdServo(){
    rc_send_servo_pulse_normalized(1, -0.5);
    //usleep(22000);
}






