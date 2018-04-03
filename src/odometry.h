#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

void initializeOdometryThread(pthread_t pThread);
void *runOdometryThread(void * param);

double getOrientation();
void setOrientation(double newTheta);
double getXPosition();
void setXPosition(double newX);
double getYPosition();
void setYPosition(double newY);
void setOdomPrintDataFlag();
void resetOdomPrintDataFlag();

#endif
