/** @file buttons.c
 *  @brief Driver for onboard buttons.
 *
 *  Functions relating to BeagleBone Blue's onbaord buttons are contained in 
 *  this source file.
 *
 *  @bug No know bugs.
 */

#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <string.h>
#include "buttons.h"
#include "fsm.h"
#include "runMode.h"

/*******************************************************************************
 *  void on_pause_pressed()
 *
 *  @brief Callback for pause button press.
 *
 *  If the user holds the pause button for 2 seconds, set state to exiting which 
 *  triggers the rest of the program to exit cleanly.
 *
 *  @return void
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
    /* Check for release 100 times in this period, over two seconds */
	const int samples = 100;	    
	const int us_wait = 2000000;
	
	/* Keep checking to see if the button is still held down */
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("Long press detected, shutting down...\n");
	rc_set_state(EXITING);

	return;
}

/*******************************************************************************
 *  void on_pause_released() 
 *	
 *  @brief Callback for pause button released.
 *
 *  Make the Pause button toggle between paused and running states.
 *
 *  @return void
*******************************************************************************/
void on_pause_released(){
	/* toggle betewen paused and running modes */
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);

	return;
}

/*******************************************************************************
 *  void on_mode_pressed() 
 *
 *  @brief Callback for mode button pressed.
 *
 *  If the user holds the mode button for 2 seconds, switch to the beginning of 
 *  the autonomous mode FSM.
 *
 *  @return void
*******************************************************************************/
void on_mode_pressed(){
    /* Check for release 100 times in this period, over 2 seconds */
	int i=0;
	const int samples = 100;	
    const int us_wait = 2000000;
	
	/* Keep checking to see if the button is still held down */
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_mode_button() == RELEASED) return;
	}
    rc_set_led(RED, OFF);
    rc_set_led(GREEN, OFF);
	printf("Resetting autonomous mode back to initial state\n");
    /* Wait until button is released to reset FSM (Already over 2 seconds) */
	while(rc_get_mode_button() == PRESSED); 
    resetFSM();

	return;
}

/*******************************************************************************
 *  void on_mode_released() 
 *
 *  @brief Callback for mode button released.
 *
 *  Make the Mode button toggle between autonomous and manual control states.
 *
 *  @return void
*******************************************************************************/
void on_mode_released(){
    static char run_mode_string[11] = "MANUAL";
	/* Toggle betewen MANUAL and AUTONOMOUS modes */
    toggleRunMode();
    strncpy(run_mode_string, (getRunMode() == AUTONOMOUS) ? "AUTONOMOUS" : "MANUAL", sizeof(run_mode_string));
    printf("Operational mode: %s\n", run_mode_string);
	return;
}

