//#include <conio.h>
//#include <fmod.h>
#include "sound.h"
#include <stdlib.h>

//FSOUND_SAMPLE* handle;

void playSound() {
/*
   // init FMOD sound system
   FSOUND_Init (44100, 32, 0);

   // load and play sample
   handle=FSOUND_Sample_Load (0,"skraa.mp3",0, 0, 0);

   FSOUND_PlaySound (0,handle);

   // clean up

   FSOUND_Sample_Free (handle);

   FSOUND_Close();
   */
    system("cd /home/robot/Beaglebot/src/ && mpg123 skraa.mp3 &");

}
