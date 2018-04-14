#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "driveTrain.h"
#include "encoders.h"
#include "fsm.h"
#include "limitSwitch.h"
#include "lineSensor.h"
#include "odometry.h"
#include "shootingMechanism.h"

static MULTI_STATE state = INIT;

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
MULTI_STATE stateTen();
MULTI_STATE stateEleven();
MULTI_STATE stateTwelve();
MULTI_STATE stateDummy();

state_ptr FUNC_LUT[NUM_STATES] =
{
    initialization,
    stateOne,
    stateTwo,
    stateThree,
    stateFour,
    stateFive,
    stateSix,
    stateSeven,
    stateEight,
    stateNine,
    stateTen,
    stateEleven,
    stateTwelve,
    stateDummy
};

void runFSM(){
    state = (FUNC_LUT[state])();
}

void resetFSM(){
    state = INIT;
}

void printFSMState(){
    printf("FSM State: %d\n", state);
}

MULTI_STATE initialization(){
    printf("State: INIT\n");
    return STATE_ONE;
}

// Drive backward until hitting the wall on the back side. (Ball gathering #1)
MULTI_STATE stateOne() {
    setSubState(FOLLOW_BACKWARD);
    if (!isBackCollision()) {
        return STATE_ONE;
    }
    setSubState(DRIVE_STOP);
    return STATE_TWO;
}

// Drive forward until hitting the wall on the front side. (Ball gathering #2)
MULTI_STATE stateTwo(){
    setSubState(FOLLOW_FORWARD);
    if (!isFrontCollision()) {
        return STATE_TWO;
    }
    // Reset for next state
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    setSubState(DRIVE_STOP);
    return STATE_THREE;
}

// Drive backward until intersection
// Turn right 90
MULTI_STATE stateThree(){
    setSubState(NOP);
    drive(-50, -50);
    usleep(1000000); // 2 Second
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    setSubState(DRIVE_STOP);
    //return STATE_DUMMY;
    return STATE_FOUR;
}

// Drive forward to center platform
MULTI_STATE stateFour() {
    setSubState(FOLLOW_FORWARD);
    usleep(3000000); // 1 Second
    setSubState(DRIVE_STOP);
    return STATE_FIVE;
}

// Align to shoot
MULTI_STATE stateFive(){
    setSubState(NOP);
    drive(-50, 50);
    usleep(1000000); // 1 Second
    setSubState(DRIVE_STOP);
    return STATE_SIX;
}

MULTI_STATE stateSix(){
    // TODO: ADD CODE TO SHOOT EVERYTHING
    setSubState(DRIVE_STOP);
    drive(0,0); setShootingMechanism();
    usleep(1000000); // 1 Second
    setShootingServo();
    usleep(5000000); // 5 Seconds
    resetShootingMechanism();

    setSubState(DRIVE_STOP);
    setSubState(DRIVE_STOP);
    return STATE_SEVEN;
}

// Align back onto platform
MULTI_STATE stateSeven(){
    setSubState(NOP);
    drive(50, -50);
    usleep(1000000); // 1 Second
    setSubState(DRIVE_STOP);
    return STATE_EIGHT;
}

// Drive back toward ball release mechanisms
MULTI_STATE stateEight(){
    if(!isBackCollision()){
        setSubState(FOLLOW_BACKWARD);
        return STATE_EIGHT;
    }
    setSubState(DRIVE_STOP);
    return STATE_NINE;
}

// Turn left 90
// Return to state 1
MULTI_STATE stateNine(){
    setSubState(TURN_POS_90);
    usleep(10000);
    return STATE_ONE;
}

MULTI_STATE stateTen(){
    return STATE_ELEVEN;
}

MULTI_STATE stateEleven(){
    return STATE_TWELVE;
}

MULTI_STATE stateTwelve(){
    return STATE_DUMMY;
}

MULTI_STATE stateDummy(){
    return STATE_DUMMY;
}
