#ifndef ENCODERS_H
#define ENCODERS_H

typedef enum ENCODER {FRONT_LEFT=1, FRONT_RIGHT=2, BACK_LEFT=3, BACK_RIGHT=4} ENCODER;

int getEncoder(int encoder);
int getLastEncoder(int encoder);
int resetEncoder(int encoder);

#endif
