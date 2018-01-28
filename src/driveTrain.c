#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include "terminus.h"
#include "driveTrain.h"

void init_drive_test(pthread_t pThread) {
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		exit(-1);
	}
    printDriveInstructions();
	pthread_create(&pThread, NULL, parseKeyboardInput, NULL);
	initializeDrivePins();
}

void initializeDrivePins(){
    int driveMotorPins[] = { MOTOR_A_0, MOTOR_A_1, MOTOR_B_0, MOTOR_B_1};
    for (int i=0; i < 4; i++) {
        rc_gpio_export(driveMotorPins[i]);
        rc_gpio_set_dir(driveMotorPins[i], OUTPUT_PIN);
    }

    // Pinmux mode to PWM
    rc_set_pinmux_mode(PWM_0A, PINMUX_PWM);
    rc_set_pinmux_mode(PWM_0B, PINMUX_PWM);

    // Initialize PWM subsystem
    rc_pwm_init(0, PWM_FREQUENCY);
}

void *parseKeyboardInput(void * param){
	while (1) {
        char nextChar = getch();
        printf("---");
        switch (nextChar) {
            case 'w':
                drive(100, 100);
                printf("Forward");
                break;
            case 's':
                drive(-100, -100);
                printf("Reverse");
                break;
            case 'd':
                drive(-100, 100);
                printf("Right Turn");
                break;
            case 'a':
                drive(100, -100);
                printf("Left Turn");
                break;
            case 'p':
                drive(0, 0);
                printf("Halt");
                break;
            case 'q':
                drive(0, 0);
                printf("System Exit---\n\n");
                exit(0);
            default:
                printf("Running Thread");
                break;
        }
        printf("\n");
	}
	return 0; // Exits void thread
}

void drive(int lSpeed, int rSpeed) {
    rc_gpio_set_value_mmap(MOTOR_A_0, lSpeed >= 0 ? LOW : HIGH);
    rc_gpio_set_value_mmap(MOTOR_A_1, lSpeed >  0 ? HIGH : LOW);

    rc_gpio_set_value_mmap(MOTOR_B_0, rSpeed >  0 ? HIGH : LOW);
    rc_gpio_set_value_mmap(MOTOR_B_1, rSpeed >= 0 ? LOW : HIGH);

    lSpeed = lSpeed > 0 ? lSpeed : -1*lSpeed;
    rSpeed = rSpeed > 0 ? rSpeed : -1*rSpeed;

    rc_pwm_set_duty_mmap(0, 'A', lSpeed > MAX_DRIVE_SPEED ? 1.0 : lSpeed/100.0);
    rc_pwm_set_duty_mmap(0, 'B', rSpeed > MAX_DRIVE_SPEED ? 1.0 : rSpeed/100.0);
}
