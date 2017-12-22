/*******************************************************************************
* beaglebot.c
*
* Code to control a modified 4 wheel drive car with the L298N motor driver.
* The controls W, A, S, D are setup the same was as in typical video games.
*******************************************************************************/

// usefulincludes is a collection of common system includes, for convenience
#include <rc_usefulincludes.h> 
// Main roboticscape API header
#include <roboticscape.h> 
// Extra utility functions implemented (e.g. getch())
#include "extra.h"

// Definitions
#define MOTOR_A_0 BLUE_GP0_PIN_3
#define MOTOR_A_1 BLUE_GP0_PIN_4
#define MOTOR_B_0 BLUE_GP0_PIN_5
#define MOTOR_B_1 BLUE_GP0_PIN_6
#define PWM_0A GPS_HEADER_PIN_3
#define PWM_0B GPS_HEADER_PIN_4
#define PWM_FREQUENCY 25000

// Motor Direction
typedef enum {NEUTRAL, FORWARD, BACK} direction;

// Function declarations
void on_pause_pressed();
void on_pause_released();
void init_msg();
void init_pins();
void test_all_motors();
void go_forward(float speed);
void go_back(float speed);
void go_sharp_left_forward(float speed);
void go_sharp_right_forward(float speed);
void go_left_forward(float speed);
void go_right_forward(float speed);
void go_sharp_left_back(float speed);
void go_sharp_right_back(float speed);
void go_left_back(float speed);
void go_right_back(float speed);
void set_A_motors(float speed, direction dir);
void set_B_motors(float speed, direction dir);

void stop_left_motors();
void stop_right_motors();
void stop_all_motors();
void estop_all_motors();
float verify_speed(float speed);
void print_state(int current_heading, int current_turning);
int get_new_input_flag();
void set_new_input_flag();
void reset_new_input_flag();

// Thread declarations
void * input_checker(void * param);

// Global Variables
static int new_input_flag = 0;

/*******************************************************************************
* int main() 
*
* This main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
	// Initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// Initialization
    init_msg();                         // Show Welcome message
    init_pins();                        // Initialze GPIOs
    // Set Pause button handlers
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

    // Initialize variables
    // Total time vehicle executes a direction command (in microseconds):
    // (latch time in microseconds) ~= sleepTimeUS * latchLoops
    int sleepTimeUS = 4000;             // Loop sleep time in microseconds
    int latchLoops = 100;               // (latch time in microseconds) / (sleepTimeUS)
    char inputCh = 0;                   // Character read
    int forward_back_tick = 0;          // Holds forward/back loop count used to check if enough time has passed
    int left_right_tick = 0;            // Holds left/right loop count used to check if enough time has passed
    int current_heading = 0;            // -1 = Back    0 = Neutral     1 = Forward
    int current_turning = 0;            // -1 = Left    0 = Neutral     1 = Right
    float max_speed = 100.0;                // Max speed of motors. 100.0
    float current_speed = 0;            // Speed of motors. 0.0 - 100.0
    //char previousCh = 0;              // Previous character read

    // Initialize pthreads
	pthread_t  input_thread;
	pthread_create(&input_thread, NULL, input_checker, (void*) &inputCh);

	// Done initializing, so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// Handle other states
		if(rc_get_state()==RUNNING){
			// Activate Green LED
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);

            // Take in terminal input without pressing enter
            //system("/bin/stty raw"); 
            //printf("Current inputCh is: %c\n", inputCh);
            
            // Update state variables based on input
            // if (inputCh != previousCh){
            if (get_new_input_flag()){
                switch(inputCh){
                    case '\033':    // Esc key
                        rc_set_state(EXITING);
                        break;
                    case 'w':       // Forward
                        current_heading = 1;
                        forward_back_tick = 0;
                        break;
                    case 's':       // Back
                        current_heading = -1;
                        forward_back_tick = 0;
                        break;
                    case 'a':       // Left
                        current_turning = -1;
                        left_right_tick = 0;
                        break;
                    case 'd':       // Right
                        current_turning = 1;
                        left_right_tick = 0;
                        break;
                    case ' ':       // E-stop
                        current_heading = 0;
                        current_turning = 0;
                        estop_all_motors();
                        break;
                    case 'p':       // Pause
                        rc_set_state(PAUSED);
                        break;
                }
                reset_new_input_flag();
            }

            // FSM for heading and turning
            switch(current_heading){
                case -1:            // Back
                    if (current_turning == -1){         // Back Left
                        go_left_back(current_speed);
                    } else if (current_turning == 0) {  // Straight Back
                        go_back(current_speed);
                    } else if (current_turning == 1) {  // Back Right
                        go_right_back(current_speed);
                    }
                    break;
                case 0:             // No heading
                    if (current_turning == -1){         // Sharp left turn
                        go_sharp_left_forward(max_speed);   // Use max speed for sharp turn
                    } else if (current_turning == 0) {  // Neutral
                        stop_all_motors();
                    } else if (current_turning == 1) {  // Sharp right turn
                        go_sharp_right_forward(max_speed);  // Use max speed for sharp turn
                    }
                    break;
                case 1:             // Forward
                    if (current_turning == -1){         // Sharp left turn
                        go_left_forward(current_speed);
                    } else if (current_turning == 0) {  // Neutral
                        go_forward(current_speed);
                    } else if (current_turning == 1) {  // Sharp right turn
                        go_right_forward(current_speed);
                    }
                    break;
            }
            /*
            switch(current_turning){
                case -1:            // Left
                    if(current_heading == 1){
                        go_left(current_speed);
                    } else {
                        go_sharp_left(current_speed);
                    }
                    break;
                case 0:             // Neutral
                    stop_left();
                    stop_right();
                    break;
                case 1:             // Right
                    stop_left();
                    go_right();
                    break;
            } */
            print_state(current_heading, current_turning);
            //previousCh = inputCh;   // Save current character
            //printf("Previous input is: %c\n", previousCh);
            // Set back
            //system("/bin/stty cooked");

            // Perform motor testing routine
            //test_all_motors();
		} else if(rc_get_state()==PAUSED){
            // Paused state
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);

            stop_all_motors();      // Set pins low

            // Take in terminal input without newline character
            //system("/bin/stty raw");
            printf("\rPAUSED          \t");
            fflush(stdout);
            if(get_new_input_flag()){
                switch(inputCh){
                    case '\033':    // Esc key
                        rc_set_state(EXITING);
                        break;
                    case 'p':       // Pause key
                        rc_set_state(RUNNING);
                        printf("\rRUNNING      \t");
                        fflush(stdout);
                        break;
                }
                reset_new_input_flag();
            }
            // Set back
            //system("/bin/stty cooked");
		}
		// Always sleep at some point
		usleep(sleepTimeUS); // Was 100000
        forward_back_tick += 1;
        left_right_tick += 1;
        // Update current speed based on forward/back tick
        // ramps down over time, as key is released
        current_speed = max_speed - forward_back_tick;
        //printf("Forward/Backward count is %d\n", forward_back_tick);
        //printf("Left/Right count is %d\n", left_right_tick);

        if (forward_back_tick >= latchLoops){
            current_heading = 0;
            forward_back_tick = 0;  
            inputCh = 0; 
        }
        if(left_right_tick >= latchLoops){
            current_turning = 0;
            left_right_tick = 0;
            inputCh = 0;
        }
	}
	
	// Exit cleanly
    stop_all_motors();                  // Set all pins low
	pthread_join(input_thread, NULL);   // Wait for thread to stop
	rc_cleanup(); 
	return 0;
}


/*******************************************************************************
* FUNCTIONS
*******************************************************************************/
/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// Toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}
/*******************************************************************************
* void on_pause_pressed() 
*
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
	const int samples = 100;	    // Check for release 100 times in this period
	const int us_wait = 2000000;    // 2 seconds
	
	// Now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples); // Check every 20ms for 2 seconds
		if(rc_get_pause_button() == RELEASED) return; // If released before 2 seconds, continue
	}
	printf("Long press detected, shutting down\n");
	rc_set_state(EXITING); 
	return;
}
/*******************************************************************************
* void init_msg() 
*
* Show welcome message.
*******************************************************************************/
void init_msg(){
	printf("\nHello from BeagleBot!\n");
	printf("-----------------------\n");
	printf("|       Controls      |\n");
	printf("-----------------------\n");
    printf("| w - Forward         |\n");
    printf("| s - Back            |\n");
    printf("| a - Left            |\n");
    printf("| d - Right           |\n");
    printf("| Spacebar - E-Stop   |\n");
    printf("| Esc - Quit          |\n");
	printf("-----------------------\n");
}
/*******************************************************************************
* void init_gpio() 
*
* Initialize GPIOs to correct state.
*******************************************************************************/
void init_pins(){
    // Export GPIOs
    rc_gpio_export(MOTOR_A_0);                      // GPIO3_17 on schematic -- Right
    rc_gpio_export(MOTOR_A_1);                      // GPIO3_20 on schematic -- Left
    rc_gpio_export(MOTOR_B_0);                      // GPIO1_17 on schematic -- Back
    rc_gpio_export(MOTOR_B_1);                      // GPIO1_25 on schematic -- Forward
    // Set GPIO Direction
    rc_gpio_set_dir(MOTOR_A_0, OUTPUT_PIN);         // GPIO3_17 on schematic -- Right
    rc_gpio_set_dir(MOTOR_A_1, OUTPUT_PIN);         // GPIO3_20 on schematic -- Left
    rc_gpio_set_dir(MOTOR_B_0, OUTPUT_PIN);         // GPIO1_17 on schematic -- Back
    rc_gpio_set_dir(MOTOR_B_1, OUTPUT_PIN);         // GPIO1_25 on schematic -- Forward
    // Change Pinmux mode to PWM
    rc_set_pinmux_mode(PWM_0A, PINMUX_PWM);        
    rc_set_pinmux_mode(PWM_0B, PINMUX_PWM);
    // Initialize PWM subsystem
    rc_pwm_init(0, PWM_FREQUENCY);                  // Subsystem 0, PWM frequency
}
/*******************************************************************************
* void test_all_motors()
*
* Routine to test all motors.
*******************************************************************************/
/*
void test_all_motors(){
    rc_gpio_set_value_mmap(FORWARD_PIN, HIGH);  // Set pin
    printf("\rFORWARD\t");
    fflush(stdout);
    usleep(1000000); // Run for 1 second
    rc_gpio_set_value_mmap(FORWARD_PIN, LOW);   // Reset pin
    usleep(500000); // Wait for half second

    rc_gpio_set_value_mmap(BACK_PIN, HIGH);     // Set pin
    printf("\rBACK   \t");
    fflush(stdout);
    usleep(1000000); // Run for 1 second
    rc_gpio_set_value_mmap(BACK_PIN, LOW);      // Reset pin
    usleep(500000); // Wait for half second

    rc_gpio_set_value_mmap(LEFT_PIN, HIGH);     // Set pin
    printf("\rLEFT   \t");
    fflush(stdout);
    usleep(1000000); // Run for 1 second
    rc_gpio_set_value_mmap(LEFT_PIN, LOW);      // Reset pin
    usleep(500000); // Wait for half second

    rc_gpio_set_value_mmap(RIGHT_PIN, HIGH);    // Set pin
    printf("\rRIGHT  \t");
    fflush(stdout);
    usleep(1000000); // Run for 1 second
    rc_gpio_set_value_mmap(RIGHT_PIN, LOW);     // Reset pin
    usleep(500000); // Wait for half second
}*/


/*******************************************************************************
* void go_forward(float speed)
*
* Activate forward drive. 
*******************************************************************************/
void go_forward(float speed){
    speed = verify_speed(speed);
    set_A_motors(speed, FORWARD);
    set_B_motors(speed, FORWARD);
}
/*******************************************************************************
* void go_back(float speed)
*
* Activate backward drive. 
*******************************************************************************/
void go_back(float speed){
    speed = verify_speed(speed);
    set_A_motors(speed, BACK);
    set_B_motors(speed, BACK);
}
/*******************************************************************************
* void go_sharp_left_forward(float speed)
*
* Activate left drive. 
*******************************************************************************/
void go_sharp_left_forward(float speed){
    speed = verify_speed(speed);
    set_A_motors(speed, FORWARD);
    set_B_motors(speed, BACK);
}
/*******************************************************************************
* void go_sharp_right_forward(float speed)
*
* Activate right drive. 
*******************************************************************************/
void go_sharp_right_forward(float speed){
    speed = verify_speed(speed);
    set_A_motors(speed, BACK);
    set_B_motors(speed, FORWARD);
}
/*******************************************************************************
* void go_left_forward(float speed)
*
* Activate left drive. 
*******************************************************************************/
void go_left_forward(float speed){
    speed = verify_speed(speed);
    set_A_motors(speed, FORWARD);
    set_B_motors(speed, NEUTRAL);
}
/*******************************************************************************
* void go_right_forward(float speed)
*
* Activate right drive. 
*******************************************************************************/
void go_right_forward(float speed){
    speed = verify_speed(speed);
    set_A_motors(speed, NEUTRAL);
    set_B_motors(speed, FORWARD);
}
/*******************************************************************************
* void go_sharp_left_back(float speed)
*
* Activate left drive. 
*******************************************************************************/
void go_sharp_left_back(float speed){
    speed = verify_speed(speed);
    set_A_motors(speed, BACK);
    set_B_motors(speed, FORWARD);
}
/*******************************************************************************
* void go_sharp_right_back(float speed)
*
* Activate right drive. 
*******************************************************************************/
void go_sharp_right_back(float speed){
    speed = verify_speed(speed);
    set_A_motors(speed, FORWARD);
    set_B_motors(speed, BACK);
}
/*******************************************************************************
* void go_left_back(float speed)
*
* Activate left drive. 
*******************************************************************************/
void go_left_back(float speed){
    speed = verify_speed(speed);
    set_A_motors(speed, BACK);
    set_B_motors(speed, NEUTRAL);
}
/*******************************************************************************
* void go_right_back(float speed)
*
* Activate right drive. 
*******************************************************************************/
void go_right_back(float speed){
    speed = verify_speed(speed);
    set_A_motors(speed, NEUTRAL);
    set_B_motors(speed, BACK);
}
/*******************************************************************************
* void set_A_motors(float speed, direction dir)
*
* Sets system A motors. (One output pair of motor driver).
* speed - value between 0 and 100
* dir - direction type: NEUTRAL, FORWARD, BACK
*******************************************************************************/
void set_A_motors(float speed, direction dir){
    switch(dir){
        case NEUTRAL:
            rc_gpio_set_value_mmap(MOTOR_A_0, LOW);
            rc_gpio_set_value_mmap(MOTOR_A_1, LOW);
            break;
        case FORWARD:
            rc_gpio_set_value_mmap(MOTOR_A_0, LOW);
            rc_gpio_set_value_mmap(MOTOR_A_1, HIGH);
            break;
        case BACK:
            rc_gpio_set_value_mmap(MOTOR_A_0, HIGH);
            rc_gpio_set_value_mmap(MOTOR_A_1, LOW);
            break;
    }
    rc_pwm_set_duty_mmap(0, 'A', speed/100.0);
}
/*******************************************************************************
* void set_B_motors(float speed, direction dir)
*
* Sets system B motors. (One output pair of motor driver).
* speed - value between 0 and 100
* dir - direction type: NEUTRAL, FORWARD, BACK
*******************************************************************************/
void set_B_motors(float speed, direction dir){
    switch(dir){
        case NEUTRAL:
            rc_gpio_set_value_mmap(MOTOR_B_0, LOW);
            rc_gpio_set_value_mmap(MOTOR_B_1, LOW);
            break;
        case FORWARD:
            rc_gpio_set_value_mmap(MOTOR_B_0, HIGH);
            rc_gpio_set_value_mmap(MOTOR_B_1, LOW);
            break;
        case BACK:
            rc_gpio_set_value_mmap(MOTOR_B_0, LOW);
            rc_gpio_set_value_mmap(MOTOR_B_1, HIGH);
            break;
    }
    rc_pwm_set_duty_mmap(0, 'B', speed/100.0);
}

/*******************************************************************************
* void stop_left_motors()
*
* Stop left drive. 
*******************************************************************************/
void stop_left_motors(){
    set_A_motors(0, NEUTRAL);
}
/*******************************************************************************
* void stop_right_motors()
*
* Stop right drive. 
*******************************************************************************/
void stop_right_motors(){
    set_B_motors(0, NEUTRAL);
}
/*******************************************************************************
* void stop_all_motors() 
*
* Stops all motors.
*******************************************************************************/
void stop_all_motors(){
    stop_left_motors();
    stop_right_motors();
}
/*******************************************************************************
* void estop_all_motors() 
*
* Emergency stop all motors. (Includes print statement)
*******************************************************************************/
void estop_all_motors(){
    stop_left_motors();
    stop_right_motors();
    printf("\rE-STOP  \t");
    fflush(stdout);
}
/*******************************************************************************
* void verify_speed() 
*
* Check speed value
*******************************************************************************/
float verify_speed(float speed){
    if(speed > 100.0){
        speed = 100.0;
    } else if(speed < 0.0) {
        speed = 0.0;
    }
    return speed;
}
/*******************************************************************************
* void print_state() 
*
* Prints currrent state based on heading and turning.
*******************************************************************************/
void print_state(int current_heading, int current_turning){
    printf("\r");
    switch(current_heading){
        case -1:
            printf("Back    ");
            break;
        case 1:
            printf("Forward ");
            break;
    }
    switch(current_turning){
        case -1:
            printf("Left            ");
            break;
        case 0:
            printf("                ");
            break;
        case 1:
            printf("Right           ");
            break;
    }
}
/*******************************************************************************
* int get_new_input_flag()
*
* Returns new input flag.
*******************************************************************************/
int get_new_input_flag(){
    return new_input_flag;
}
/*******************************************************************************
* void set_new_input_flag()
*
* Sets new input flag.
*******************************************************************************/
void set_new_input_flag(){
    new_input_flag = 1;
}
/*******************************************************************************
* void reset_new_input_flag()
*
* Resets new input flag.
*******************************************************************************/
void reset_new_input_flag(){
    new_input_flag = 0;
}
/*******************************************************************************
 *  THREADS
*******************************************************************************/
/*******************************************************************************
* void * input_checker(char * inputCh)
*
* Takes new character input.
*******************************************************************************/
void * input_checker(void * param){
    char * inputCh = (char *) param;
    char newCh;
	while(rc_get_state()!=EXITING){
        newCh = getch();
        *inputCh = newCh;
        set_new_input_flag();
        //printf("New input: %c\n", newCh);
        usleep(10000);
	}
	return NULL;
}
