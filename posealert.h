/*
 * posealert.h
 *
 * Created: 2024/4/22 23:08:09
 *  Author: tobby
 */ 


#ifndef POSEALERT_H_
#define POSEALERT_H_

#define NECK_ANGLE_THRESHOLD 25.0
#define BACK_ANGLE_THRESHOLD 25.0
#define LUMBAR_TWIST_THRESHOLD 35.0
#define TILT_THRESHOLD 30.0
#define PELVIC_THRESHOLD 20.0

int checkPosture(double roll1, double pitch1, double yaw1, double roll2, double pitch2, double yaw2,
double roll3, double pitch3, double yaw3, double roll4, double pitch4, double yaw4);

void printAlert(int poseCode);

#endif /* POSEALERT_H_ */