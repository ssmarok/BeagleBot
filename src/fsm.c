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
    stateTwelve
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

// Drive backward until detecting a full wall of black line on front sensor. Then, turn right 90 degrees
MULTI_STATE stateThree(){
    // Make sure robot backs up enough before line sensors take over. Encoders were reset in STATE_THREE
    // After minimum distance back traveled, line sensors code segment takes over as encoder is not reset in this state
    setSubState(FOLLOW_BACKWARD);
    if(getEncoder(FRONT_LEFT_ENCODER) > -4700 || !isFullLineBack()){
        // Note: updateLineData is implicit in FOLLOW_BACKWARD so isFullLineBack() below is accurate
        return STATE_THREE;
    }
    printf("Encoder value, State 4: %d\n", getEncoder(FRONT_LEFT_ENCODER));

    /* TODO: Do we need this? Can it just be replaced by what is above?
    int *backSensor;
    backSensor = getBackLineSensor();
    int count = 0;
    // Check first 4 sensors
    int i;
    for(i = 0; i < LINE_SENSOR_LEN; i++){
        count = count + *(backSensor + i);
    }
    
    if(count < 3){
        setSubState(FOLLOW_BACKWARD);
        return STATE_FOUR;
    }
    */
    printOutLineData();
    setSubState(TURN_NEG_90);
    setSubState(DRIVE_STOP);
    return STATE_FOUR;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor.
// #2 Drive forward until detecting a full line on the BACK line sensor.
// This gets past the little line on the middle platform of the field
MULTI_STATE stateFour() {
    setSubState(FOLLOW_FORWARD);
    if (!isFrontCollision()) {
        return STATE_FOUR;
    }
    printOutLineData();
    setSubState(DRIVE_STOP);
    return STATE_FIVE;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor. Then, turn right 90 degrees
MULTI_STATE stateFive(){
    setSubState(TURN_NEG_90);
    setSubState(DRIVE_STOP);
    return STATE_SIX;
}

// Drive forward until hitting the wall on the front side. (Ball gathering #4)
MULTI_STATE stateSix(){
    setSubState(FOLLOW_FORWARD);
    if (!isFrontCollision()) {
        return STATE_SIX;
    }
    setSubState(DRIVE_STOP);
    return STATE_SEVEN;
}

// Drive backward until hitting the wall on the back side. (Ball gathering #3)
MULTI_STATE stateSeven(){
    setSubState(FOLLOW_BACKWARD);
    if (!isBackCollision()) {
        return STATE_SEVEN;
    }
    // Reset for next state
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    setSubState(DRIVE_STOP);
    return STATE_EIGHT;
}

// Drive backward until detecting a full wall of black line on front sensor. Then, turn right (approximately 0-15 degrees).
// This should be tested brute force for the optimal angle
MULTI_STATE stateEight(){
    // Make sure robot backs up enough before line sensors take over.
    // Encoders were reset in STATE_THREE
    // After minimum distance back traveled, line sensors code segment takes over
    // as encoder is not reset in this state
    setSubState(FOLLOW_FORWARD);
    if(getEncoder(FRONT_LEFT_ENCODER) < 4700 || !isFullLineBack()){
        return STATE_EIGHT;
    }
    /*
    printf("Encoder value, State 4: %d\n", getEncoder(FRONT_LEFT_ENCODER));

    int *backSensor;
    backSensor = getBackLineSensor();
    int count = 0;
    // Check first 4 sensors
    int i;
    for(i = 0; i < LINE_SENSOR_LEN; i++){
        count = count + *(backSensor + i);
    }
    
    if(count < 3){
        setSubState(FOLLOW_BACKWARD);
        return STATE_EIGHT;
    }
    printOutLineData();
    */

    //TODO: PUT CORRECT TURN ANGLE
    setSubState(TURN_TO_SHOOT);
    usleep(10000);
    return STATE_NINE;
}

// SHOOT ALL THE BALLS (includes necessary wait). Then, turn 90 degrees.
MULTI_STATE stateNine(){
    setSubState(DRIVE_STOP);
    drive(0,0);
    return STATE_TEN; 
}

// SHOOT ALL THE BALLS (includes necessary wait). Then, turn 90 degrees.
MULTI_STATE stateTen(){
    //printf("State: TEN\n");
    setSubState(DRIVE_STOP);
    drive(0,0);

    setShootingMechanism();
    usleep(1000000); // 1 Second
    setShootingServo();
    usleep(5000000); // 5 Seconds
    resetShootingMechanism();

    setSubState(TURN_TO_ALIGN);
    setSubState(DRIVE_STOP);
    usleep(100000);
    return STATE_ELEVEN;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor.
// #2 Drive forward until detecting a full line on the BACK line sensor.
// This gets past the little line on the middle platform of the field
// Reset to state #1
MULTI_STATE stateEleven(){
    //printf("State: ELEVEN\n");
    setSubState(DRIVE_STOP);
    drive(0,0);
    usleep(10000);
    return STATE_ELEVEN;//TODO: CHANGE TO STATE_ONE_
    /*
    if (!isFrontCollision()) {
        setSubState(FOLLOW_FORWARD);
        return STATE_ELEVEN;
    }
    // Reset for next state
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    setSubState(DRIVE_STOP);
    return STATE_TWELVE;
    */
}
