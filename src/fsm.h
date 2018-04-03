#ifndef FSM_H
#define FSM_H

// FSM States
typedef enum MULTI_STATE{
    INIT=0,
    STATE_ONE=1,
    STATE_TWO=2,
    STATE_THREE=3,
    STATE_FOUR=4,
    STATE_FIVE=5,
    STATE_SIX=6,
    STATE_SEVEN=7,
    STATE_EIGHT=8,
    STATE_NINE=9,
    STATE_TEN=10,
    STATE_ELEVEN=11,
    NUM_STATES=12
}MULTI_STATE;

typedef MULTI_STATE (*state_ptr)( void );

typedef int DRIVESTATE;
// ** STARTS at -1
enum DRIVESTATE { NOP=-1, DRIVE_STOP=0, FOLLOW_FORWARD, FOLLOW_BACKWARD, TURN_NEG_90, TURN_POS_90 };

void runFSM();
void resetFSM();
void printFSMState();

#endif
