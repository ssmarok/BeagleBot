#include <rc_usefulincludes.h>
#include "fsm.h"
#include "lineSensor.h"
#include "driveTrain.h"
#include "shootingMechanism.h"

typedef int DRIVESTATE;
enum DRIVESTATE { FOLLOW_FORWARD, FOLLOW_BACKWARD, TURN_POS_90, TURN_NEG_90 };

MULTI_STATE initialization();
MULTI_STATE stateOne();
MULTI_STATE stateTwo();
MULTI_STATE stateThree();
MULTI_STATE stateFour();

state_ptr FUNC_LUT[NUM_STATES] =
{
    initialization,
    stateOne,
    stateTwo,
    stateThree,
    stateFour
};

void runFSM(){
    static MULTI_STATE state = INIT;
    usleep(10000);
    state = (FUNC_LUT[state])();
}

MULTI_STATE initialization(){
    //putchar('i');
    return STATE_ONE;
}

MULTI_STATE stateOne() {
    setSubState(1);
    if (!isFullLine()) {
        return STATE_ONE;
    }
    setSubState(0);
    return STATE_TWO;
}

MULTI_STATE stateTwo() {
    //putchar('2');
    drive(0,0);
    return STATE_THREE;
}

MULTI_STATE stateThree(){
    //putchar('3');
    return STATE_FOUR;
}

MULTI_STATE stateFour(){
    //putchar('4');
    usleep(10000);
    return STATE_FOUR;
}

