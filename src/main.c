#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "driveTrain.h"
#include "fsm.h"

int main(){
    pthread_t pThread = 0;
    pthread_t subPThread = 0;
    pthread_t shootPThread = 0;

    /*
     * Implicitly loops in parallel
     * Initializes Motor GPIO Pins
     * Uses stdio file for manual testing of drive
     */
    initializeDriveTest(pThread);
    initializeSubState(subPThread);
    initializeShootState(shootPThread);

    while(1) {
        /*
         * Runs empty FSM functions
         * TODO: Fill in FSM
         * Fills stdio buffer with current state
         * TODO: Delete stdio access when no longer necessary (FSM filled)
         */
        runFSM();
    }
	pthread_join(pThread, NULL);
	pthread_join(subPThread, NULL);
	pthread_join(shootPThread, NULL);
    rc_cleanup();
    return 0;
}

