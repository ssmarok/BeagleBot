#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "driveTrain.h"

int main(){
    pthread_t pThread = 0;
    init_drive_test(pThread);
    while(1) {
    }
    return 0;
}

