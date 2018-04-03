#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <string.h>
#include "buttons.h"
#include "fsm.h"
#include "runMode.h"

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
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("Long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
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
* void on_mode_pressed() 
*
* If the user holds the hold button for 2 seconds, switch to the beginning of 
* the autonomous mode FSM
*******************************************************************************/
void on_mode_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_mode_button() == RELEASED) return;
	}
	printf("Resetting autonomous mode back to initial state\n");
    // TODO: RESET FSM HERE
	while(rc_get_mode_button() == PRESSED); // Wait until button is released to reset FSM (Already over 2 seconds)
    resetFSM();
	return;
}
/*******************************************************************************
* void on_mode_released() 
*	
* Make the Mode button toggle between autonomous and manual control states.
*******************************************************************************/
void on_mode_released(){
    static char run_mode_string[11] = "MANUAL";
	// Toggle betewen MANUAL and AUTONOMOUS modes
    toggleRunMode();
    strncpy(run_mode_string, (getRunMode() == AUTONOMOUS) ? "AUTONOMOUS" : "MANUAL", sizeof(run_mode_string));
    printf("Current BeagleBot operational mode: %s\n", run_mode_string);
	return;
}

