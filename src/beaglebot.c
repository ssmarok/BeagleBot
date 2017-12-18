/*******************************************************************************
* beaglebot.c
*
* Test file for BeagleBot
*******************************************************************************/

// usefulincludes is a collection of common system includes, for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>

// Definitions
#define FORWARD_PIN BLUE_GP0_PIN_3
#define BACK_PIN BLUE_GP0_PIN_4
#define LEFT_PIN BLUE_GP0_PIN_5
#define RIGHT_PIN BLUE_GP0_PIN_6

// Function declarations
void on_pause_pressed();
void on_pause_released();
void test_all_motors();

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
	printf("\nHello from BeagleBone Blue!\n");
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);
    // Setup GPIOs
    rc_gpio_set_dir(FORWARD_PIN, OUTPUT_PIN); // GPIO1_25 on schematic -- Forward
    rc_gpio_set_dir(BACK_PIN, OUTPUT_PIN); // GPIO1_17 on schematic -- Back
    rc_gpio_set_dir(LEFT_PIN, OUTPUT_PIN); // GPIO3_20 on schematic -- Left
    rc_gpio_set_dir(RIGHT_PIN, OUTPUT_PIN); // GPIO3_17 on schematic -- Right

	// Done initializing, so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// Handle other states
		if(rc_get_state()==RUNNING){
			// Do things
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
            // Perform motor testing routine
            test_all_motors();
		}
		else if(rc_get_state()==PAUSED){
            // Paused state
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		// Always sleep at some point
		usleep(100000);
	}
	
	// Exit cleanly
	rc_cleanup(); 
	return 0;
}


/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// toggle betewen paused and running modes
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
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples); // Check every 20ms for 2 seconds
		if(rc_get_pause_button() == RELEASED) return; // if released before 2 seconds, continue
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING); 
	return;
}
/*******************************************************************************
* void test_all_motors() 
*
* Routine to test all motors.
*******************************************************************************/
void test_all_motors(){
    rc_gpio_set_value_mmap(FORWARD_PIN, HIGH); // Turn on motor
    usleep(1000000); // Run for 1 second
    rc_gpio_set_value_mmap(FORWARD_PIN, LOW); // Turn off motor
    usleep(500000); // Wait for half second

    rc_gpio_set_value_mmap(BACK_PIN, HIGH); // Turn on motor
    usleep(1000000); // Run for 1 second
    rc_gpio_set_value_mmap(BACK_PIN, LOW); // Turn off motor
    usleep(500000); // Wait for half second

    rc_gpio_set_value_mmap(LEFT_PIN, HIGH); // Turn on motor
    usleep(1000000); // Run for 1 second
    rc_gpio_set_value_mmap(LEFT_PIN, LOW); // Turn off motor
    usleep(500000); // Wait for half second

    rc_gpio_set_value_mmap(RIGHT_PIN, HIGH); // Turn on motor
    usleep(1000000); // Run for 1 second
    rc_gpio_set_value_mmap(RIGHT_PIN, LOW); // Turn off motor
    usleep(500000); // Wait for half second
}
