#include <rc_usefulincludes.h>
#include <roboticscape.h>

void initLimitSwitches(void) {
}

// 0 = switch open
// else switch closed
int isSwitchClosed(int limitSwitch) {
    return 1;
}

int isFrontCollision(void) {
    return isSwitchClosed(0) && isSwitchClosed(1);
}

int isBackCollision(void) {
    return isSwitchClosed(2) && isSwitchClosed(3);
}
