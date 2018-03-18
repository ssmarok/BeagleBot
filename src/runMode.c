#include "runMode.h"

// Create operational mode variable
static RUN_MODE runMode = MANUAL;

int getRunMode(){
    return runMode;
}

void toggleRunMode() {
    runMode = (runMode == AUTONOMOUS) ? MANUAL : AUTONOMOUS;
}

void setRunMode(RUN_MODE desiredRunMode) {
    runMode = desiredRunMode;

}
