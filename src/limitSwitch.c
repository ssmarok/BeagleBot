#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "limitSwitch.h"
#include "pinMap.h"

void initLimitSwitches(void) {
    rc_set_pinmux_mode(FRONT_LEFT_LIMIT, PINMUX_GPIO_PU);
    rc_set_pinmux_mode(FRONT_RIGHT_LIMIT, PINMUX_GPIO_PU);
    rc_set_pinmux_mode(BACK_LEFT_LIMIT, PINMUX_GPIO_PU);
    rc_set_pinmux_mode(BACK_RIGHT_LIMIT, PINMUX_GPIO_PU);
}

// 1 = switch open, since pins pulled up
// else switch closed
int isSwitchClosed(int limitSwitch) {
    return !(rc_gpio_get_value_mmap(limitSwitch));
}

int isFrontCollision(void) {
    return isSwitchClosed(FRONT_LEFT_LIMIT) && isSwitchClosed(FRONT_RIGHT_LIMIT);
}

int isBackCollision(void) {
    return isSwitchClosed(BACK_LEFT_LIMIT) && isSwitchClosed(BACK_RIGHT_LIMIT);
}


void printOutLimitSwitchData(void) {
    int limitSwitchArray[4] = {FRONT_LEFT_LIMIT, FRONT_RIGHT_LIMIT, BACK_LEFT_LIMIT, BACK_RIGHT_LIMIT};
    printf("\n");
    for (int i=0; i<2; i++) {
        printf("[%d]%c", isSwitchClosed(limitSwitchArray[i]), i!= 1 ? '-' : ' ');
    }
    printf("}: Front Limit Switches\n{ ");
    for (int i=2; i<4; i++) {
        printf("[%d]%c", isSwitchClosed(limitSwitchArray[i]), i!= 3 ? '-' : ' ');
    }
    printf("}: Back Limit Switches\n");
}
