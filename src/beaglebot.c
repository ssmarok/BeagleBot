/*******************************************************************************
* beaglebot.c
*
* Code to control a modified RC car.
*******************************************************************************/

// usefulincludes is a collection of common system includes, for convenience
#include <rc_usefulincludes.h> 
// Main roboticscape API header
#include <roboticscape.h> 
// Extra utility functions implemented (e.g. getch())
#include "extra.h"

// Definitions
#define FORWARD_PIN BLUE_GP0_PIN_6
#define BACK_PIN BLUE_GP0_PIN_5
#define LEFT_PIN BLUE_GP0_PIN_4
#define RIGHT_PIN BLUE_GP0_PIN_3

// Function declarations
void on_pause_pressed();
void on_pause_released();
void init_msg();
void init_gpio();
void test_all_motors();
void go_forward();
void go_back();
void go_left();
void go_right();
void stop_forward();
void stop_back();
void stop_left();
void stop_right();
void stop_all_motors();
void estop_all_motors();
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
* This template main function contains these critical components
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
    init_gpio();                        // Initialze GPIOs
    // Set Pause button handlers
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

    // Initialize variables
    // Total time vehicle executes a direction command (in microseconds):
    // (latch time in microseconds) ~= sleepTimeUS * latchLoops
    int sleepTimeUS = 10000;            // Loop sleep time in microseconds
    int latchLoops = 40;                // (latch time in microseconds) / (sleepTimeUS)
    char inputCh = 0;                   // Character read
    int forward_back_tick = 0;          // Holds forward/back loop count used to check if enough time has passed
    int left_right_tick = 0;            // Holds left/right loop count used to check if enough time has passed
    int current_heading = 0;            // -1 = Back    0 = Neutral     1 = Forward
    int current_turning = 0;            // -1 = Left    0 = Neutral     1 = Right
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

            // Dual FSM for heading and turning
            switch(current_heading){
                case -1:            // Back
                    stop_forward();
                    go_back();
                    break;
                case 0:             // Neutral
                    stop_forward();
                    stop_back();
                    break;
                case 1:             // Forward
                    stop_back();
                    go_forward();
                    break;
            }
            switch(current_turning){
                case -1:            // Left
                    stop_right();
                    go_left();
                    break;
                case 0:             // Neutral
                    stop_left();
                    stop_right();
                    break;
                case 1:             // Right
                    stop_left();
                    go_right();
                    break;
            }
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
void init_gpio(){
    // Export GPIOs
    rc_gpio_export(FORWARD_PIN);                // GPIO1_25 on schematic -- Forward
    rc_gpio_export(BACK_PIN);                   // GPIO1_17 on schematic -- Back
    rc_gpio_export(LEFT_PIN);                   // GPIO3_20 on schematic -- Left
    rc_gpio_export(RIGHT_PIN);                  // GPIO3_17 on schematic -- Right
    // Set GPIO Direction
    rc_gpio_set_dir(FORWARD_PIN, OUTPUT_PIN);   // GPIO1_25 on schematic -- Forward
    rc_gpio_set_dir(BACK_PIN, OUTPUT_PIN);      // GPIO1_17 on schematic -- Back
    rc_gpio_set_dir(LEFT_PIN, OUTPUT_PIN);      // GPIO3_20 on schematic -- Left
    rc_gpio_set_dir(RIGHT_PIN, OUTPUT_PIN);     // GPIO3_17 on schematic -- Right
}
/*******************************************************************************
* void test_all_motors()
*
* Routine to test all motors.
*******************************************************************************/
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
}

/*******************************************************************************
* void go_forward() 
*
* Activate forward drive. 
*******************************************************************************/
void go_forward(){
    rc_gpio_set_value_mmap(FORWARD_PIN, HIGH);  // Set pin
}
/*******************************************************************************
* void go_back() 
*
* Activate backward drive. 
*******************************************************************************/
void go_back(){
    rc_gpio_set_value_mmap(BACK_PIN, HIGH);     // Set pin 
}
/*******************************************************************************
* void go_left() 
*
* Activate left drive. 
*******************************************************************************/
void go_left(){
    rc_gpio_set_value_mmap(LEFT_PIN, HIGH);     // Set pin
}
/*******************************************************************************
* void go_right() 
*
* Activate right drive. 
*******************************************************************************/
void go_right(){
    rc_gpio_set_value_mmap(RIGHT_PIN, HIGH);    // Set pin
}
/*******************************************************************************
* void stop_forward() 
*
* Stop forward drive. 
*******************************************************************************/
void stop_forward(){
    rc_gpio_set_value_mmap(FORWARD_PIN, LOW);  // Reset pin
}
/*******************************************************************************
* void stop_back() 
*
* Stop backward drive. 
*******************************************************************************/
void stop_back(){
    rc_gpio_set_value_mmap(BACK_PIN, LOW);     // Reset pin 
}
/*******************************************************************************
* void stop_left() 
*
* Stop left drive. 
*******************************************************************************/
void stop_left(){
    rc_gpio_set_value_mmap(LEFT_PIN, LOW);     // Reset pin
}
/*******************************************************************************
* void stop_right() 
*
* Stop right drive. 
*******************************************************************************/
void stop_right(){
    rc_gpio_set_value_mmap(RIGHT_PIN, LOW);    // Reset pin
}
/*******************************************************************************
* void stop_all_motors() 
*
* Stops all motors.
*******************************************************************************/
void stop_all_motors(){
    rc_gpio_set_value_mmap(FORWARD_PIN, LOW);   // Reset pin
    rc_gpio_set_value_mmap(BACK_PIN, LOW);      // Reset pin
    rc_gpio_set_value_mmap(LEFT_PIN, LOW);      // Reset pin
    rc_gpio_set_value_mmap(RIGHT_PIN, LOW);     // Reset pin
}
/*******************************************************************************
* void estop_all_motors() 
*
* Emergency stop all motors. (Includes print statement)
*******************************************************************************/
void estop_all_motors(){
    rc_gpio_set_value_mmap(FORWARD_PIN, LOW);   // Reset pin
    rc_gpio_set_value_mmap(BACK_PIN, LOW);      // Reset pin
    rc_gpio_set_value_mmap(LEFT_PIN, LOW);      // Reset pin
    rc_gpio_set_value_mmap(RIGHT_PIN, LOW);     // Reset pin
    printf("\rE-STOP  \t");
    fflush(stdout);
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
