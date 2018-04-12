#include <rc_usefulincludes.h>
#include <roboticscape.h>

// TODO: Make static 
int encoderHistory[] = {0, 0, 0, 0};
int leftFrontEncoder = 0;
int rightFrontEncoder = 0;

int getEncoder(int encoder){
    int value = rc_get_encoder_pos(encoder);  // Channels start at 1
    encoderHistory[encoder] = value;
    return value;
}

int getLastEncoder(int encoder){
    return encoderHistory[encoder];
}

int resetEncoder(int encoder){
    return rc_set_encoder_pos(encoder, 0);
}

