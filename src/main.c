#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <string.h>
#include "buttons.h"
#include "driveTrain.h"
#include "fsm.h"
#include "imu.h"
#include "limitSwitch.h"
#include "odometry.h"
#include "runMode.h"
#include "shootingMechanism.h"
#include "terminus.h"

#define DEBUG 1


int main(){
    // Initialize robotics cape first
	if(rc_initialize()){
		fprintf(stderr,"Initialization failed. Are you root?\n");
		exit(-1);
	}
    // Declare pThreads
    pthread_t keyboardThread = 0;
    pthread_t drivePThread = 0;
    pthread_t shootPThread = 0;
    pthread_t odomPThread = 0;
    // Initialize Pins, Switches, and IMU
	initializeDrivePins();
    initLimitSwitches();
    initializeIMU();
    // Setup button interrupt functions
    rc_set_pause_pressed_func(&on_pause_pressed);
    rc_set_pause_released_func(&on_pause_released);
    rc_set_mode_pressed_func(&on_mode_pressed);
    rc_set_mode_released_func(&on_mode_released);
    // Initialize Threads
    initializeDriveThread(drivePThread);
    initializeServoThread(shootPThread);
    initializeKeyboardThread(keyboardThread);
    initializeOdometryThread(odomPThread);
    // Print Instructions
    printDriveInstructions();
    // Set state to running after finished initialization
	rc_set_state(RUNNING); 

    /* Main Program Loop */
    while(rc_get_state()!=EXITING) {
        if(rc_get_state()==RUNNING){
            if(getRunMode() == MANUAL){
                rc_set_led(GREEN, OFF);
                rc_set_led(RED, OFF);
            } else if(getRunMode() == AUTONOMOUS){
                rc_set_led(GREEN, ON);
                rc_set_led(RED, OFF);
                /*
                 * Runs empty FSM functions
                 * TODO: Fill in FSM
                 * Fills stdio buffer with current state
                 * TODO: Delete stdio access when no longer necessary (FSM filled)
                 */
                runFSM();
            }

        } else if (rc_get_state() == PAUSED){
            rc_set_led(GREEN, OFF);
            rc_set_led(RED, ON); 
        }
        // Sleep at some point
        usleep(10000);
    }

    // Cleanup threads
    pthread_join(keyboardThread, NULL);
	pthread_join(drivePThread, NULL);
	pthread_join(shootPThread, NULL);
	pthread_join(odomPThread, NULL);
    // Cleanup and reset
    rc_power_off_imu();
    resetTermios();
    rc_cleanup();
    exit(0);
    return 0;
}

