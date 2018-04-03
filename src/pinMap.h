#ifndef PINMAP_H
#define PINMAP_H

// Drive Train Definitions
#define MOTOR_A_0 BLUE_GP0_PIN_3
#define MOTOR_A_1 BLUE_GP0_PIN_4
#define PWM_0A GPS_HEADER_PIN_3
#define PWM_0B GPS_HEADER_PIN_4
#define PWM_FREQUENCY 20000

// Shooting Mechanism Definitions
#define MOTOR_FIRE BLUE_GP0_PIN_5

// Limit Switch Definitions
#define FRONT_LEFT_LIMIT BLUE_SPI_PIN_6_SS2 
#define FRONT_RIGHT_LIMIT SPI_HEADER_PIN_5 
#define BACK_LEFT_LIMIT SPI_HEADER_PIN_4
#define BACK_RIGHT_LIMIT SPI_HEADER_PIN_3

#endif
