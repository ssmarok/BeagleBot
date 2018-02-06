#include <rc_usefulincludes.h>
#include "fsm.h"

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
    //putchar('1');
    return STATE_TWO;
}

MULTI_STATE stateTwo() {
    //putchar('2');
    return STATE_THREE;
}

MULTI_STATE stateThree(){
    //putchar('3');
    return STATE_FOUR;
}

MULTI_STATE stateFour(){
    //putchar('4');
    return STATE_ONE;
}

