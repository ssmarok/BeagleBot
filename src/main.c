#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <string.h>
#include "driveTrain.h"
#include "shootingMechanism.h"
#include "limitSwitch.h"
#include "fsm.h"
#include "terminus.h"
#include "buttons.h"
#include "runMode.h"

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

    // Initialize Pins, Switches, and Buttons
	initializeDrivePins();
    initLimitSwitches();
    rc_set_pause_pressed_func(&on_pause_pressed);
    rc_set_pause_released_func(&on_pause_released);
    rc_set_mode_pressed_func(&on_mode_pressed);
    rc_set_mode_released_func(&on_mode_released);

    // Initialize Threads
    initializeDriveThread(drivePThread);
    initializeServoThread(shootPThread);
    initializeKeyboardThread(keyboardThread);

    // Print Instructions
    printDriveInstructions();

    // Set state to running after finished initialization
	rc_set_state(RUNNING); 

    /*
     * Implicitly loops in parallel
     * Initializes Motor GPIO Pins
     * Uses stdio file for manual testing of drive
     */
    while(rc_get_state()!=EXITING) {
    if (DEBUG) {
        // Pins for debugging and taking in keyboard input
        initializeDriveTest(pThread);
    }
    else {
        // TODO: Put this stuff into initializaDrivePins and possible rename it
        if(rc_initialize()){
            fprintf(stderr,"Initialization failed. Are you root?\n");
            exit(-1);
        }
        initializeDrivePins();
    }
    // Other initializations
    initLimitSwitches();

    // Initialize Threads
    initializeDriveThread(drivePThread);
    initializeServoThread(shootPThread);

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
                // runFSM();
            }

        } else if (rc_get_state() == PAUSED){
            rc_set_led(GREEN, OFF);
            rc_set_led(RED, ON); }
        // Sleep at some point
        usleep(10000);
    }
    // Cleanup threads
    pthread_join(keyboardThread, NULL);
	pthread_join(drivePThread, NULL);
	pthread_join(shootPThread, NULL);
    resetTermios();
    rc_cleanup();
    exit(0);
    return 0;
}

