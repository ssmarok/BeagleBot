/** @file driveTrain.c
 *  @brief Driver for the drive system.
 *
 *  Functions relating to robots drive system are contained in 
 *  this source file.
 *
 *  @bug No know bugs.
 */

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

/* Global SUBSTATE for drive thread substate */
int SUBSTATE = 0;

/*******************************************************************************
 *  void initializeDriveThread(pthread_t pThread)
 *
 *  @brief Initialize the Drive thread.
 *
 *  Create a new thread that handles the drive states.
 *
 *  @param pThread - the pThread instance.
 *
 *  @return void
*******************************************************************************/
void initializeDriveThread(pthread_t pThread) {
	pthread_create(&pThread, NULL, runDriveThread, NULL);
}

/*******************************************************************************
 *  void initializeKeyboardThread(pthread_t pThread)
 *
 *  @brief Initialize the Keyboard thread.
 *
 *  Create a new thread that handles keyboard input.
 *
 *  @param pThread - the pThread instance.
 *
 *  @return void
*******************************************************************************/
void initializeKeyboardThread(pthread_t pThread) {
	pthread_create(&pThread, NULL, parseKeyboardInput, NULL);
}

/*******************************************************************************
 *  void initializeDrivePins()
 *
 *  @brief Initialize pins for the driving the robot.
 *
 *  GPIO pins for the wheels motor driver as well as the shooting motor are 
 *  initialized. PWM pins for the wheels motor driver are are also initialized.
 *  The servo power rail is activated. The IR sensor initialization routine is
 *  also called.
 *
 *  @return void
*******************************************************************************/
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

/*******************************************************************************
 *  void setSubState(int subState)
 *
 *  @brief Set the driving substate.
 *
 *  Update the driving substate.
 *
 *  DRIVESTATE used in fsm.c. Commendted here for reference
 *  enum DRIVESTATE { NOP, DRIVE_STOP, FOLLOW_FORWARD, FOLLOW_BACKWARD, 
 *                  TURN_POS_90, TURN_NEG_90 };
 *
 *  @param subState - the new substate.
 *
 *  @return void
*******************************************************************************/
void setSubState(int subState) {
    SUBSTATE = subState;
    /* Print out subState name */
    /*
    static const char *subStateNames[] = {
        "NOP", "DRIVE_STOP", "FOLLOW_FORWARD", "FOLLOW_BACKWARD", "TURN_NEG_90", "TURN_POS_90"
    }; 
    // +1 for -1 case
    printf("DRIVESTATE: %s SUBSTATE: %d \n", subStateNames[subState+1], subState);  
    */
}

/*******************************************************************************
 *  void *runDriveThread(void * param)
 *
 *  @brief Function in which the drive thread loops.
 *
 *  Function continuously loops to keep the drive thread active. Substate is 
 *  used to determine how the robot will drive.
 *
 *  @param *param - Initial thread parameter.
 *
 *  @return void
*******************************************************************************/
void *runDriveThread(void * param) {
    while (1) {
        switch (SUBSTATE) {
            case -1:    /* DRIVESTATE: NOP */
                break;
            case 0:     /* DRIVESTATE: DRIVE_STOP */
                drive(0, 0);
                break;
            case 1:     /* DRIVESTATE: FOLLOW_FORWARD */
                //lineFollowForward();
                lineFollowForwardFast();
                break;
            case 2:     /* DRIVESTATE: FOLLOW_BACKWARD */
                lineFollowBackward();
                break;
            case 3:     /* DRIVESTATE: TURN_POS_90 (LEFT) */
                turnLeft90();
                SUBSTATE = 0;
                break;
            case 4:     /* DRIVESTATE: TURN_NEG_90 (RIGHT) */
                turnRight90();
                SUBSTATE = 0;
                break;
            default:    /* Sleep */
                usleep(2000);
                break;
        }
    /* Always sleep for some time */
	usleep(50);
    }
    /* Exit void thread */
	return 0; 
}

/*******************************************************************************
 *  void *parseKeyboardInput(void * param)
 *
 *  @brief Function in which the keyboard thread loops.
 *
 *  Function continuously loops to keep the keyboard thread active. Character 
 *  input is used to determine the action the robot will perform.
 *
 *  @param *param - Initial thread parameter.
 *
 *  @return void
*******************************************************************************/
void *parseKeyboardInput(void * param){
	while (1) {
        /* Get character input */
        char nextChar = getch();
        printf("---");
        switch (nextChar) {
            case 'r': /* Reset encoders */
                resetEncoder(FRONT_LEFT_ENCODER);
                resetEncoder(FRONT_RIGHT_ENCODER);
                break;
            case 'w': /* Drive forward (line following) */
                printf("Forward");
                SUBSTATE = 1;
                break;
            case 'W': /* Drive forward (no line following) */
                drive(BASE_SPEED, BASE_SPEED);
                SUBSTATE = -1;
                break;
            case 's': /* Drive backwards (line following) */
                printf("Reverse");
                SUBSTATE = 2;
                break;
            case 'S': /* Drive backwards (no line following) */
                drive(-1*BASE_SPEED, -1*BASE_SPEED);
                SUBSTATE = -1;
                break;
            case 'd': /* Turn right (90 degrees) */
                printf("Right Turn");
                SUBSTATE = 4;
                break;
            case 'D': /* Turn right (no limit) */
                drive(MAX_SPEED, -1*MAX_SPEED);
                SUBSTATE = -1;
                break;
            case 'a': /* Turn left (90 degrees) */
                printf("Left Turn");
                SUBSTATE = 3;
                break;
            case 'A': /* Turn left (no limit) */
                drive(-1*MAX_SPEED, MAX_SPEED);
                SUBSTATE = -1;
                break;
            case 'P': /* Halt motors - reset FSM */
                drive(0, 0);
                printf("Halt");
                SUBSTATE = 0;
                setRunMode(MANUAL);
                resetFSM();
                break;
            case 'q': /* Quit */
                drive(0, 0);
                printf("System Exit---\n\n");
                SUBSTATE = 0;
	            rc_set_state(EXITING); 
                rc_cleanup();
                exit(0);
            case 'y': /* Send negative pulse to servos (for testing) */
                rc_send_servo_pulse_normalized_all(-1.0);
                usleep(10000);
                break;
            case 't': /* Send positive pulse to servos (for testing) */
                rc_send_servo_pulse_normalized_all(1.0);
                usleep(10000);
                break;
            case 'f': /* Turn on shooting motor */
                printf("FIRE");
                setShootingMechanism();
                usleep(1000000);
                setShootingServo();
                break;
            case 'h': /* Turn off shooting motor */
                printf("HOLD");
                resetShootingMechanism();
                break;
            case 'e': /* Print encoder values */
                printf("Front Left: %d\t", getEncoder(FRONT_LEFT_ENCODER));
                printf("Front Right: %d\t", getEncoder(FRONT_RIGHT_ENCODER));
                break;
            case 'l': /* Print out line sensor data */
                printOutLineData();
                break;
            case 'i': /* Print out limit switch data */ 
                printOutLimitSwitchData();
                break;
            case 'o': /* Print out odometry data */
                setOdomPrintDataFlag();
                break;
            case 'x': /* Print out FSM state */
                printFSMState();
                break;
            case 'c': /* Clear terminal */
                printf("\033[0;0H");
                printf("\033[2J");
                break;
            case '1': /* Set to manual mode */
                setRunMode(MANUAL);
                printf("Operational Mode: Manual");
                break;
            case '2': /* Set to autonomous mode */
                setRunMode(AUTONOMOUS);
                printf("Operational Mode: Autonomous");
                break;
            case '?': /* Print out help menu */
                printDriveInstructions();
                break;
            case '~': /* Run Init State */
                break;
            case '!': /* Run State 1 */
                break;
            case '@': /* Run State 2 */
                break;
            case '#': /* Run State 3 */
                break;
            case '$': /* Run State 4 */
                break;
            case '%': /* Run State 5 */
                break;
            case '^': /* Run State 6 */
                break;
            case '&': /* Run State 7 */
                break;
            case '*': /* Run State 8 */
                break;
            case '(': /* Run State 9 */
                break;
            case ')': /* Run State 10 */
                break;
            default:  /* Any other key press */
                printf("Running Thread");
                break;
        }
        printf("\n");
        /* Always sleep for some time */
        usleep(5000);
	}
    /* Exit void thread */
	return 0; 
}

/*******************************************************************************
 *  void turnLeft90()
 *
 *  @brief Turn left 90 degrees using IMU data.
 *
 *  The IMU angular data is used to turn 90 degrees (left) at fixed 
 *  wheel speeds.
 * 
 *  @return void
*******************************************************************************/
void turnLeft90(){
    double start_orientation = 0; 
    start_orientation = getOrientation();
    /* 1.56 is ~90 degrees in radians */
    while(getOrientation() < start_orientation + 1.6) { 
        drive(-70, 70);
    }
}

/*******************************************************************************
 *  void turnRight90()
 *
 *  @brief Turn right 90 degrees using IMU data.
 *
 *  The IMU angular data is used to turn -90 degrees (right) at fixed 
 *  wheel speeds.
 * 
 *  @return void
*******************************************************************************/
void turnRight90(){
    double start_orientation = 0; 
    start_orientation = getOrientation();
    /* 1.56 is ~90 degrees in radians */
    while(getOrientation() > start_orientation - 1.65) { 
        drive(40, -90);
    }
}

/*******************************************************************************
 *  int bufferSpeed(int speed, int buffer)
 *
 *  @brief Returns a speed value by comparing desired speed with buffer speed.
 *
 *  The speed is set to the buffer speed if a value less than the buffer value 
 *  is passed in.
 *
 *  @param speed  - Desired speed value
 *  @param buffer - Minimum speed value 
 *
 *  @return speed - Modified speed value
*******************************************************************************/
int bufferSpeed(int speed, int buffer) {
    /* Do not modify speed if 0 is passed in. (Halt state) */
    if (speed == 0) {
        return speed;
    }
    /* Compare speed to the positive buffer speed value. */
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

/*******************************************************************************
 *  void drive(int lSpeed, int rSpeed)
 *
 *  @brief Set the drive speed for the robot.
 *
 *  Desired wheel speed values are compared with minimum buffer speed value. 
 *  Motor controller GPIO pins are modified according to desired wheel 
 *  direction. Motors are sent a PWM signal scaling to wheel speed values.
 *
 *  @param lSpeed - Desired left wheel speed value
 *  @param rSpeed - Desired right wheel speed value
 *
 *  @return void
*******************************************************************************/
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

