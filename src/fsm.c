#include <rc_usefulincludes.h>
#include "fsm.h"
#include "lineSensor.h"
#include "driveTrain.h"
#include "shootingMechanism.h"
#include "limitSwitch.h"

typedef int DRIVESTATE;
enum DRIVESTATE { NOP, DRIVE_STOP, FOLLOW_FORWARD, FOLLOW_BACKWARD, TURN_NEG_90, TURN_POS_90 };

MULTI_STATE initialization();
MULTI_STATE stateOne();
MULTI_STATE stateTwo();
MULTI_STATE stateThree();
MULTI_STATE stateFour();
MULTI_STATE stateFive();
MULTI_STATE stateSix();
MULTI_STATE stateSeven();
MULTI_STATE stateEight();
MULTI_STATE stateNine();
MULTI_STATE stateTen ();

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

// Drive forward until detecting a full wall of black line on front sensor. Then, turn right 90 degrees
MULTI_STATE stateOne() {
    setSubState(FOLLOW_FORWARD);
    if (!isFullLineFront()) {
        turnRight90();
        return STATE_ONE;
    }
    setSubState(DRIVE_STOP);
    return STATE_TWO;
}

// Drive backward until hitting the wall on the back side. (Ball gathering #1)
MULTI_STATE stateTwo() {
    setSubState(FOLLOW_BACKWARD);
    if (!isBackCollision()) {
        return STATE_TWO;
    }
    drive(0,0);
    return STATE_THREE;
}

// Drive forward until hitting the wall on the front side. (Ball gathering #2)
MULTI_STATE stateThree(){
    if (!isFrontCollision()) {
        turnRight90();
        return STATE_ONE;
    }
    return STATE_FOUR;
}

// Drive backward until detecting a full wall of black line on front sensor. Then, turn right 90 degrees
MULTI_STATE stateFour(){
    //putchar('4');
    usleep(10000);
    return STATE_FIVE;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor.
// #2 Drive forward until detecting a full line on the BACK line sensor.
// This gets past the little line on the middle platform of the field
MULTI_STATE stateFive(){
    return STATE_SIX;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor. Then, turn right 90 degrees
MULTI_STATE stateSix(){
    return STATE_SEVEN;
}

// Drive backward until hitting the wall on the back side. (Ball gathering #3)
MULTI_STATE stateSeven(){
    return STATE_EIGHT;
}

// Drive forward until hitting the wall on the front side. (Ball gathering #4)
MULTI_STATE stateEight(){
    return STATE_NINE;
}

// Drive backward until detecting a full wall of black line on front sensor. Then, turn right (approximately 0-15 degrees).
// This should be tested brute force for the optimal angle
MULTI_STATE stateNine(){
    return STATE_TEN;
}

// SHOOT ALL THE BALLS (includes necessary wait). Then, turn 90 degrees.
MULTI_STATE stateTen(){
    return STATE_ELEVEN;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor.
// #2 Drive forward until detecting a full line on the BACK line sensor.
// This gets past the little line on the middle platform of the field
// Reset to state #1
MULTI_STATE stateEleven(){
    return STATE_ONE;
}

