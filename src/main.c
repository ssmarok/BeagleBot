#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "driveTrain.h"
#include "shootingMechanism.h"
#include "fsm.h"

#define DEBUG 1

int main(){
    pthread_t pThread = 0;
    pthread_t drivePThread = 0;
    pthread_t shootPThread = 0;

    /*
     * Implicitly loops in parallel
     * Initializes Motor GPIO Pins
     * Uses stdio file for manual testing of drive
     */
    if (DEBUG) {
        initializeDriveTest(pThread);
    }
    else {
        if(rc_initialize()){
            fprintf(stderr,"Initialization failed. Are you root?\n");
            exit(-1);
        }
        initializeDrivePins();
    }
    initializeDriveThread(drivePThread);
    initializeServoThread(shootPThread);

    while(1) {
        /*
         * Runs empty FSM functions
         * TODO: Fill in FSM
         * Fills stdio buffer with current state
         * TODO: Delete stdio access when no longer necessary (FSM filled)
         */
        if (!DEBUG) {
            runFSM();
        }
    }
    if (DEBUG) {
        pthread_join(pThread, NULL);
    }
	pthread_join(drivePThread, NULL);
	pthread_join(shootPThread, NULL);
    rc_cleanup();
    return 0;
}

