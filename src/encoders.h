#ifndef ENCODERS_H
#define ENCODERS_H

typedef enum ENCODER {FRONT_LEFT_ENCODER=1, FRONT_RIGHT_ENCODER=2, BACK_LEFT_ENCODER=3, BACK_RIGHT_ENCODER=4} ENCODER;

int getEncoder(int encoder);
int getLastEncoder(int encoder);
int resetEncoder(int encoder);

#endif
