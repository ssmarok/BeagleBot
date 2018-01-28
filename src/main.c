#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "driveTrain.h"
#include "fsm.h"

int main(){
    pthread_t pThread = 0;

    /*
     * Implicitly loops in parallel
     * Initializes Motor GPIO Pins
     * Uses stdio file for manual testing of drive
     */
    initializeDriveTest(pThread);

    while(1) {
        /*
         * Runs empty FSM functions
         * TODO: Fill in FSM
         * Fills stdio buffer with current state
         * TODO: Delete stdio access when no longer necessary (FSM filled)
         */
        runFSM();
    }
    return 0;
}

