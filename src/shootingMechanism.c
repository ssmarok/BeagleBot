#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "pinMap.h"
#include "shootingMechanism.h"

int shootState = 0;

void initializeServoThread(pthread_t pThread) {
	pthread_create(&pThread, NULL, runServoThread, NULL);
}


void *runServoThread(void * param) {
    static int servoDirection = 1;
    static uint64_t startTime = 0;
    while(1) {
        switch(shootState){
            case 0:
                holdTrigger();
                holdServo();
                break;
            case 1: // Startup shooting motor
                releaseTrigger();
                //usleep(1000000);
                break;
            case 2: // (Re)start shooting motor, and move servos
                releaseTrigger();
                releaseServo();
                releaseBucket(servoDirection);
                shootState = 3; // Move to next servo state
                break;
            case 3: // Toggle shooting motor direction -- stays in this state until firing stops
                //printf("Start time: %d\n", startTime);
                if(rc_nanos_since_boot() - startTime > 2000000000){
                    startTime = rc_nanos_since_boot();
                    servoDirection = !servoDirection;
                    //printf("Changing direction, now is: %s\n", servoDirection ? "FORWARD" : "BACKWARD");
                }
                releaseBucket(servoDirection);
                break;
        }
        usleep(SERVO_SLEEP_TIME);
    }
    return 0;
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
void releaseServo(){ rc_send_servo_pulse_normalized(TUBE_SERVO_CHANNEL, -1.4); }
void holdServo(){ rc_send_servo_pulse_normalized(TUBE_SERVO_CHANNEL, -0.70); }

void releaseBucket(int direction){ rc_send_servo_pulse_normalized(BUCKET_SERVO_CHANNEL, direction ? 1.5:-1.5); }
