#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "pinMap.h"
#include "shootingMechanism.h"

int shootState = 0;

void *runServoThread(void * param) {
    while(1) {
        switch(shootState){
            case 0:
                holdTrigger();
                holdServo();
                break;
            case 1:
                releaseTrigger();
                //usleep(1000000);
                break;
            case 2: 
                releaseTrigger();
                releaseServo();
                // IMPORTANT SHOOT
                releaseBucket();
                //usleep(10000);
                break;
        }
        usleep(SERVO_SLEEP_TIME);
    }
    return 0;
}

void initializeServoThread(pthread_t pThread) {
	pthread_create(&pThread, NULL, runServoThread, NULL);
}

void setShootingServo() {
    //usleep(1000000);
    shootState = 2;
}
void setShootingMechanism() {
    shootState = 1;
}

void resetShootingMechanism() {
    shootState = 0;
}

void releaseTrigger() { rc_gpio_set_value_mmap(MOTOR_FIRE, HIGH); }
void holdTrigger() { rc_gpio_set_value_mmap(MOTOR_FIRE, LOW); }
void releaseServo(){ rc_send_servo_pulse_normalized(1, -1); }
void holdServo(){ rc_send_servo_pulse_normalized(1, -0.60); }

void releaseBucket(){ rc_send_servo_pulse_normalized(7, 1.5); }
