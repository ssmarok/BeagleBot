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

int leftEncoderWaypoint = 0;
int rightEncoderWaypoint = 0;

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
};

void runFSM(){
    //usleep(10000);
    state = (FUNC_LUT[state])();
}

void resetFSM(){
    state = INIT;
}

void printFSMState(){
    printf("FSM State: %d\n", state);
}

MULTI_STATE initialization(){
    //putchar('i');
    printf("State: INIT\n");
    return STATE_ONE;
}

// Drive forward until detecting a full wall of black line on front sensor. Then, turn right 90 degrees
MULTI_STATE stateOne() {
    //printf("State: ONE\n");
    setSubState(FOLLOW_FORWARD);
    if (!isFullLineFront()) { 
        //turnRight90(); // Put out of this function?
        return STATE_ONE;
    }
    //rotateCW(); 
    setSubState(TURN_POS_90);
    //setSubState(DRIVE_STOP);
    return STATE_TWO;
}

// Drive backward until hitting the wall on the back side. (Ball gathering #1)
MULTI_STATE stateTwo() {
    //printf("State: TWO\n");
    leftEncoderWaypoint = getEncoder(FRONT_LEFT_ENCODER);
    rightEncoderWaypoint = getEncoder(FRONT_RIGHT_ENCODER);
    setSubState(FOLLOW_BACKWARD);
    if (!isBackCollision()) {
        return STATE_TWO;
    }
    drive(0,0);
    return STATE_THREE;
}

// Drive forward until hitting the wall on the front side. (Ball gathering #2)
MULTI_STATE stateThree(){
    //printf("State: THREE\n");
    if (!isFrontCollision()) {
        setSubState(FOLLOW_FORWARD);
        //turnRight90();      // Put out of this function?
        //return STATE_ONE;
        return STATE_THREE;
    }
    drive(0,0);
    // Reset for next state
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_FOUR;
}

// Drive backward until detecting a full wall of black line on front sensor. Then, turn right 90 degrees
MULTI_STATE stateFour(){
    //drive(0,0);
    //printf("State: FOUR\n");
    if(getEncoder(FRONT_LEFT_ENCODER) > -8200){
        setSubState(FOLLOW_BACKWARD);
        return STATE_FOUR;
    }
    /*
    if(getEncoder(FRONT_LEFT_ENCODER) > leftEncoderWaypoint || getEncoder(FRONT_RIGHT_ENCODER) > rightEncoderWaypoint){
        return STATE_FOUR;
    } */
    /*
    if (!isFullLineFront()) { 
        setSubState(FOLLOW_BACKWARD);
        return STATE_FOUR;
    }
    */
    setSubState(TURN_POS_90);
    //setSubState(DRIVE_STOP);
    usleep(100000);
    return STATE_FIVE;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor.
// #2 Drive forward until detecting a full line on the BACK line sensor.
// This gets past the little line on the middle platform of the field
MULTI_STATE stateFive(){
    //printf("State: FIVE\n");
    if (!isFullLineFront()) { 
        setSubState(FOLLOW_FORWARD);
        return STATE_FIVE;
    }
    drive(0,0);
    setSubState(DRIVE_STOP);
    return STATE_SIX;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor. Then, turn right 90 degrees
MULTI_STATE stateSix(){
    //printf("State: SIX\n");
    drive(0,0);
    setSubState(DRIVE_STOP);
    usleep(100000);
    return STATE_SIX; // TODO: Change back STATE_SEVEN
}

// Drive backward until hitting the wall on the back side. (Ball gathering #3)
MULTI_STATE stateSeven(){
    //printf("State: SEVEN\n");
    return STATE_EIGHT;
}

// Drive forward until hitting the wall on the front side. (Ball gathering #4)
MULTI_STATE stateEight(){
    //printf("State: EIGHT\n");
    return STATE_NINE;
}

// Drive backward until detecting a full wall of black line on front sensor. Then, turn right (approximately 0-15 degrees).
// This should be tested brute force for the optimal angle
MULTI_STATE stateNine(){
    //printf("State: NINE\n");
    return STATE_TEN;
}

// SHOOT ALL THE BALLS (includes necessary wait). Then, turn 90 degrees.
MULTI_STATE stateTen(){
    //printf("State: TEN\n");
    return STATE_ELEVEN;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor.
// #2 Drive forward until detecting a full line on the BACK line sensor.
// This gets past the little line on the middle platform of the field
// Reset to state #1
MULTI_STATE stateEleven(){
    //printf("State: ELEVEN\n");
    return STATE_ONE;
}

// Reset to state #1
