#include <termios.h>
#include <stdio.h>
#include "terminus.h"

static struct termios old, new;

/* Initialize new terminal i/o settings */
void initTermios(int echo){
	tcgetattr(0, &old); /* grab old terminal i/o settings */
	new = old; /* make new settings same as old settings */
	new.c_lflag &= ~ICANON; /* disable buffered i/o */
	new.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
	tcsetattr(0, TCSANOW, &new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void){
	tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo){
	char ch;
	initTermios(echo);
	ch = getchar();
	resetTermios();
	return ch;
}

/* Read 1 character without echo */
char getch(void){
	return getch_(0);
}

/* Read 1 character with echo */
char getche(void){
	return getch_(1);
}

void printDriveInstructions(){
	printf("\nHello from BeagleBot!\n");
	printf("-----------------------\n");
	printf("|       Controls      |\n");
	printf("-----------------------\n");
	printf("                       \n");
	printf("---------Drive---------\n");
    printf("| w - Forward         |\n");
    printf("| s - Back            |\n");
    printf("| a - Left            |\n");
    printf("| d - Right           |\n");
    printf("| P - STOP ALL        |\n");
	printf("---------Shoot---------\n");
    printf("| f - Fire            |\n");
    printf("| h - Stop Firing     |\n");
	printf("---------Debug---------\n");
    printf("| e - Encoders        |\n");
    printf("| l - Line Sensors    |\n");
    printf("| i - Limit Switches  |\n");
	printf("---------System---------\n");
    printf("| c - Clear           |\n");
    printf("| q - Sys Exit        |\n");
	printf("-----------------------\n");
}

