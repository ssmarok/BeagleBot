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
MULTI_STATE stateThirteen();
MULTI_STATE stateAlignFirst();
MULTI_STATE stateAlignSecond();
MULTI_STATE stateBackAlign();
MULTI_STATE stateFirstCorrection();
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
    stateThirteen,
    stateAlignFirst,
    stateAlignSecond,
    stateBackAlign,
    stateFirstCorrection,
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
    setSubState(BACKWARD);
    if (!isBackCollision()) {
        return STATE_ONE;
    }
    setSubState(DRIVE_STOP);
    // Reset for next state
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    setSubState(DRIVE_STOP);
    return STATE_FIRST_CORRECTION;
}

// Drive forward until hitting the wall on the front side. (Ball gathering #2)
MULTI_STATE stateTwo(){
    setSubState(FORWARD);
    if (!isFrontCollision()) {
        return STATE_TWO;
    }
    // Reset for next state
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    setSubState(DRIVE_STOP);
    return STATE_THREE;
}

// Drive backward using encoder values, then turn 90 degrees
MULTI_STATE stateThree(){
    // Make sure robot backs up enough before line sensors take over. Encoders were reset in STATE_TWO
    // After minimum distance back traveled, line sensors code segment takes over as encoder is not reset in this state
    if(getEncoder(FRONT_LEFT_ENCODER) > -4670){
        setSubState(BACKWARD);
        // Note: updateLineData is implicit in FOLLOW_BACKWARD so isFullLineBack() below is accurate
        usleep(1000);
        return STATE_THREE;
    } 
    /*
    else if (!isHalfLineBack()) {
        return STATE_THREE;
    }
    */
   
    printf("Encoder value, State 4: %d\n", getEncoder(FRONT_LEFT_ENCODER));
    printOutLineData();

    setSubState(TURN_NEG_90);
    usleep(10000);

    /*
    setSubState(FORWARD);
    usleep(1000000);
    */

    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    setSubState(DRIVE_STOP);
    //return STATE_DUMMY;
    return STATE_FOUR;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor.
// #2 Drive forward until detecting a full line on the BACK line sensor.
// This gets past the little line on the middle platform of the field
MULTI_STATE stateFour() {
    // Drive for a bit using line following
    if(getEncoder(FRONT_LEFT_ENCODER) < 15000){
        setSubState(FOLLOW_FORWARD);
        return STATE_FOUR;
    }

    setSubState(FORWARD);

    // Check if wall has been hit
    if (!isFrontCollision()) {
        // Once full line is detected on other side, change to following forwaed
        // blindly
        setSubState(FORWARD); //TODO
        return STATE_FOUR;
    }
    printOutLineData();

    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    setSubState(DRIVE_STOP);
    return STATE_FIVE;
}

// Turn right 90 degrees
MULTI_STATE stateFive(){
    setSubState(TURN_NEG_90);
    usleep(10000);
    setSubState(DRIVE_STOP);

    // TODO: SEE IF THIS WORKS
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_ALIGN_FIRST; // Extra alignment
    // return STATE_SIX
}

// Drive backward until hitting the wall on the back side. (Ball gathering #3)
MULTI_STATE stateSix(){
    if (!isBackCollision()) {
        //setSubState(FOLLOW_BACKWARD);
        setSubState(BACKWARD); // TODO
        return STATE_SIX;
    }

    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER); 
    setSubState(DRIVE_STOP);
    return STATE_SEVEN;
}

// Drive forward until hitting the wall on the front side. (Ball gathering #4)
MULTI_STATE stateSeven(){
    //printf("In forward state\n");
    if(getEncoder(FRONT_RIGHT_ENCODER) < 2000){
        setSubState(NOP);
        drive(-1 * BASE_SPEED, BASE_SPEED);
        return STATE_SEVEN;
    }
    setSubState(DRIVE_STOP);
    usleep(10000);
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_EIGHT;
}

// Drive backward until detecting a full wall of black line on front sensor. Then, turn right (approximately 0-15 degrees).
// This should be tested brute force for the optimal angle
MULTI_STATE stateEight(){
    // Make sure robot backs up enough before line sensors take over.
    // Encoders were reset in STATE_SEVEN
    // After minimum distance back traveled, line sensors code segment takes over
    // as encoder is not reset in this state
    /*
    if(getEncoder(FRONT_LEFT_ENCODER) > -7700){
        //setSubState(FOLLOW_BACKWARD);
        setSubState(BACKWARD);
        return STATE_EIGHT;
    }

    //TODO: PUT CORRECT TURN ANGLE
    setSubState(TURN_TO_SHOOT);
    usleep(10000);
    */
    if (!isFrontCollision()) {
       // setSubState(FOLLOW_FORWARD);
        setSubState(FORWARD);
        return STATE_EIGHT;
    }
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);

    setSubState(DRIVE_STOP);
    usleep(10000);
    drive(0,0);
    return STATE_NINE;

}

MULTI_STATE stateNine(){

    if(getEncoder(FRONT_LEFT_ENCODER) > -2000){
        setSubState(FOLLOW_BACKWARD);
        return STATE_NINE;
    }

    //resetEncoder(FRONT_LEFT_ENCODER);
    //resetEncoder(FRONT_RIGHT_ENCODER);

    setSubState(DRIVE_STOP);
    setSubState(TURN_TO_SHOOT);
    usleep(10000);
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_TEN;
}
// SHOOT ALL THE BALLS (includes necessary wait). Then, turn 90 degrees.
MULTI_STATE stateTen(){
    setSubState(DRIVE_STOP);
    drive(0,0); setShootingMechanism();
    usleep(1000000); // 1 Second
    setShootingServo();
    usleep(9000000); // 5 Seconds
    resetShootingMechanism();

    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    setSubState(DRIVE_STOP);
    return STATE_ELEVEN;
}

// Go back to center
MULTI_STATE stateEleven(){
    if(getEncoder(FRONT_LEFT_ENCODER) > -5150){
        //setSubState(FOLLOW_BACKWARD);
        setSubState(FOLLOW_BACKWARD);
        return STATE_ELEVEN;
    } 
    /*else if (!isHalfLineBack()) {
        return STATE_TEN;
    }
    */

    /*
    setSubState(TURN_TO_ALIGN);
    */
    usleep(10000);
    setSubState(DRIVE_STOP);
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_TWELVE;
}

// #1 Drive forward until detecting a full line on the FRONT line sensor.
// #2 Drive forward until detecting a full line on the BACK line sensor.
// This gets past the little line on the middle platform of the field
MULTI_STATE stateTwelve(){
    //while(1) { //TODO: REMOVE LOOP & CONTINUE TO STATE_TWELVE
    //}
    setSubState(TURN_NEG_90);
    usleep(10000);
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);

    setSubState(NOP);
    usleep(10000);

    /*
    setSubState(FORWARD);
    usleep(2000000);
    */

    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_THIRTEEN;
}

// Drive back toward the other side of the field
// Reset to state #1
MULTI_STATE stateThirteen(){
    //static int fullLineDetected = 0;

    // Drive for a bit using line following
    if(getEncoder(FRONT_LEFT_ENCODER) < 15000){
        setSubState(FOLLOW_FORWARD); // TODO
        return STATE_THIRTEEN;
    }

    setSubState(FORWARD);
    // Once full line is detected on other side, change to following forwaed
    // blindly
    //setSubState(FORWARD);

    // Once full line is detected on other side, change to following forward
    // blindly
    /*
    if(isFullLineFront() || fullLineDetected){
        printf("FULL LINE WAS DETECTED\n");
        fullLineDetected = 1;
        setSubState(FORWARD);
    } else {
        setSubState(FOLLOW_FORWARD);
    }
    if (!isFrontCollision()) {
        return STATE_TWELVE;
    }
    */
    if (!isFrontCollision()) {
        // Once full line is detected on other side, change to following forwaed
        // blindly
        setSubState(FORWARD);
        return STATE_THIRTEEN;
    }
    setSubState(TURN_NEG_90);
    usleep(10000);
    setSubState(DRIVE_STOP);
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_ALIGN_SECOND;
}

// After getting to second side
MULTI_STATE stateAlignFirst(){
    printf("In align state\n");
    setSubState(TURN_TO_ALIGN);
    usleep(40000);
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_SIX;
}

// After getting to start
MULTI_STATE stateAlignSecond(){
    printf("In second align state\n");
    setSubState(TURN_TO_ALIGN);
    usleep(40000);
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_BACK_ALIGN;
}

MULTI_STATE stateBackAlign(){
    if (!isBackCollision()) {
        //setSubState(FOLLOW_BACKWARD);
        setSubState(BACKWARD); // TODO
        usleep(10000);
        return STATE_BACK_ALIGN;
    }

    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    setSubState(DRIVE_STOP);
    return STATE_ONE;
}

MULTI_STATE stateFirstCorrection(){
    //printf("In forward state\n");
    if(getEncoder(FRONT_RIGHT_ENCODER) < 2000){
        setSubState(NOP);
        drive(-1 * BASE_SPEED, BASE_SPEED);
        return STATE_FIRST_CORRECTION;
    }
    setSubState(DRIVE_STOP);
    usleep(10000);
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_TWO;
}
MULTI_STATE stateDummy(){
    //setSubState(TURN_NEG_90);
    drive(0,0);
    setSubState(DRIVE_STOP);
    usleep(10000);
    resetEncoder(FRONT_LEFT_ENCODER);
    resetEncoder(FRONT_RIGHT_ENCODER);
    return STATE_DUMMY;
}
