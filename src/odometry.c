#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <math.h>
#include "odometry.h"
#include "imu.h"
#include "encoders.h"

static int odomPrintData = 0;

// Global Pose variables
double x_pos = 0;   // Meters
double y_pos = 0;   // Meters
double theta = 0;   // Radians

void initializeOdometryThread(pthread_t pThread) {
	pthread_create(&pThread, NULL, runOdometryThread, NULL);
}

void *runOdometryThread(void * param) {
    // Robot specs
    double wheel_radius = 0.045; // 90mm wheels
    double wheel_circumference = 2*PI*wheel_radius; // 2*pi*r
    double encoder_resolution = 50*64;
    //const double k = 0.14605 + 0.1524; //the sum of the distance between the wheel's x-coord and the origin, and the y-coord and the origin (11.5" by 12" robot) --> (0.2921m by 0.3048m)
    double dist_per_tick = wheel_circumference / encoder_resolution; // .000088

    // Time variables
    uint64_t last_time = 0;
    uint64_t current_time = 0;
    double dt_seconds = 0;

    // IMU data
    rc_imu_data_t *imu_data;
    float imu_gyro_z;

    // Encoder data
    int previous_encoder_values[2] = {0, 0}; //(FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL}
    int new_encoder_values[2] = {0, 0};

    // Velocity based on encoders
    double vel_x = 0;
    double vel_y = 0;

    while(1){
        if (getIMUNewDataFlag()){ 
            current_time = rc_nanos_since_boot();
            dt_seconds = (current_time - last_time)/1000000000.0;

            imu_data = getIMUData();
            imu_gyro_z = imu_data->gyro[2] * DEG_TO_RAD; // Z-axis - in radians (angular velocity)
            // Clear out noisy measurements
            if(imu_gyro_z > -0.03 && imu_gyro_z < 0.03){
                imu_gyro_z = 0;
            } 
           /* else if(odomPrintData){
                printf("IMU Val: %f\n", imu_gyro_z);
            }*/ 

            new_encoder_values[0] = getEncoder(FRONT_LEFT_ENCODER);
            new_encoder_values[1] = getEncoder(FRONT_RIGHT_ENCODER);
            if (new_encoder_values[0] > new_encoder_values[1] +10 || new_encoder_values[0] < new_encoder_values[1] - 10){
                if(odomPrintData){
                    printf("Encoder values differ: %d, %d \n", new_encoder_values[0], new_encoder_values[1]);
                }
            }

            // TODO: verify absolute value needed for turns (Ex: left wheels going forward, right going backward)
            // Maybe don't need absolute values since the x position should stay same if robot spinning in place
            int delta_front_left_encoder = new_encoder_values[0] - previous_encoder_values[0];
            int delta_front_right_encoder = new_encoder_values[1] - previous_encoder_values[1];

            double v_front_left_wheel = (delta_front_left_encoder * dist_per_tick)/dt_seconds;
            double v_front_right_wheel = (delta_front_right_encoder * dist_per_tick)/dt_seconds;

            // TODO: Don't think they can be averaged. (Ex: turning again. should be 2x vel since wheels going in opposite direction) .... But should be averaged for straight... See ex code (omni)
            // Should only count on angular velocity though then, since would be spinning in circle
            vel_x = (v_front_left_wheel + v_front_right_wheel)/2;
            // vel_y = 0; (vel_y is always 0)

            double delta_theta = imu_gyro_z * dt_seconds;
            double delta_x = (vel_x * cos(theta) - 
                   vel_y * sin(theta)) * dt_seconds;
            double delta_y = (vel_x * sin(theta) +
                   vel_y * cos(theta)) * dt_seconds;

            // Update global pose variables
            x_pos = x_pos + delta_x;
            y_pos = y_pos + delta_y;
            theta = theta + delta_theta;

            previous_encoder_values[0] = new_encoder_values[0];
            previous_encoder_values[1] = new_encoder_values[1];
            last_time = current_time;
            resetIMUNewDataFlag();

            if(odomPrintData){
                printf("dt_seconds\t\t| imu_gyro_z\t\t| Delta_Left_Encoder \t| Delta Right Encoder\t| V_Left_Wheel\t| V_Right_Wheel\t| vel_x\t| vel_y\t| delta_theta\t| delta_x\t| delta_y\t|\n");
                printf("%f\t\t| %f\t\t| %d\t\t| %d\t\t| %f\t| %f\t| %f\t| %f\t| %f\t| %f\t| %f\t|\n", dt_seconds, imu_gyro_z, delta_front_left_encoder, delta_front_right_encoder, v_front_left_wheel, v_front_right_wheel, vel_x, vel_y, delta_theta, delta_x, delta_y);
                printf("x_pos\t\t\t| y_pos\t\t| theta\t\t\t|\n");
                printf("%f\t\t| %f\t\t| %f\t\t|\n", x_pos, y_pos, theta);

                resetOdomPrintDataFlag();
            }
        }
    }
    usleep(2500);
}

double getOrientation(){
    return theta;
}

void setOrientation(double newTheta){
    theta = newTheta;
}

double getXPosition(){
    return x_pos;
}

void setXPosition(double newX){
    x_pos = newX;
}

double getYPosition(){
    return y_pos;
}

void setYPosition(double newY){
    y_pos = newY;
}

void setOdomPrintDataFlag(){
    odomPrintData = 1;
}

void resetOdomPrintDataFlag(){
    odomPrintData = 0;
}
