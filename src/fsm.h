#ifndef FSM_H
#define FSM_H

typedef enum MULTI_STATE{
    INIT=0, STATE_ONE=1, STATE_TWO=2, STATE_THREE=3, STATE_FOUR=4, NUM_STATES=5
}MULTI_STATE;

typedef MULTI_STATE (*state_ptr)( void );

void runFSM();

#endif
