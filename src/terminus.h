#ifndef TERMINUS_H
#define TERMINUS_H

void initTermios(int echo);
void resetTermios(void);
char getch_(int echo);
char getch(void);
char getche(void);
void printDriveInstructions();

#endif
