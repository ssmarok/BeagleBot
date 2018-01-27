#include <stdio.h>
typedef enum MULTI_STATE{
    INIT=0, STATE_ONE=1, STATE_TWO=2, STATE_THREE=3, STATE_FOUR=4, NUM_STATES=5
}MULTI_STATE;

MULTI_STATE init_state();
MULTI_STATE state_one();
MULTI_STATE state_two();
MULTI_STATE state_three();
MULTI_STATE state_four();

typedef MULTI_STATE (*state_ptr)( void );

state_ptr FUNC_LUT[NUM_STATES] =
{
    init_state,
    state_one,
    state_two,
    state_three,
    state_four
};


void robot_fsm(){
    static MULTI_STATE state = INIT;
    state = (FUNC_LUT[state])();
}

MULTI_STATE init_state(){
    putchar('i');
    return STATE_ONE;
}

MULTI_STATE state_one() {
    putchar('1');
    return STATE_TWO;
}

MULTI_STATE state_two() {
    putchar('2');
    return STATE_THREE;
}

MULTI_STATE state_three(){
    putchar('3');
    return STATE_FOUR;
}

MULTI_STATE state_four(){
    putchar('4');
    return STATE_ONE;
}


int main() {
    while(1) {
        robot_fsm();
    }
    return 1;
}
