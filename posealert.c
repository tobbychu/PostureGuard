/*
 * posealert.c
 *
 * Created: 2024/4/22 23:07:15
 *  Author: tobby
 */ 
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include "posealert.h"
#include "uart.h"

/*
return value:
0 - Nothing detected / good posture
1 - Neck angle is too flat!
2 - ??
3 - Hunch back.
4 - Abnormal lumbar posture.
5 - Body tilted sideways.

*/
int checkPosture(double roll1, double pitch1, double yaw1, double roll2, double pitch2, double yaw2, 
double roll3, double pitch3, double yaw3, double roll4, double pitch4, double yaw4) {
	double neckAngle = roll1;
	double upperBackAngle = -roll2;
	double lowerBackAngle = -roll3;
	double lumbarAngle = -roll4;

	// Check if the neck angle is too flat compared to the other sensors
	if (neckAngle < upperBackAngle - NECK_ANGLE_THRESHOLD) {
		//||fabs(neckAngle - lowerBackAngle) > NECK_ANGLE_THRESHOLD excluded because dont think it wrks
		return 1;
	}

	// hunch back
	if (upperBackAngle < lumbarAngle - BACK_ANGLE_THRESHOLD 
		|| lowerBackAngle - lumbarAngle < -BACK_ANGLE_THRESHOLD) {
		return 3;
	}
	
	//pelvic
	if (fabs(lumbarAngle) < 90.0 - PELVIC_THRESHOLD) {
		return 2;
	}
	
	/*
	// Abnormal lumbar
	if (fabs(pitch4 - pitch2) > LUMBAR_TWIST_THRESHOLD) {
		return 4;
	}

	// tilted body
	double rollDifference = fabs(yaw1 - yaw2);
	if (rollDifference > TILT_THRESHOLD) {
		return 5;
	}
	*/
	
	return 0;
}

void printAlert(int poseCode) {
	char alertBuff[50];
	if (poseCode == 1) {
		sprintf(alertBuff, "Posture Alert: Neck angle is too flat!\n");
		UART_putstring(alertBuff);
	}
	if (poseCode == 3) {
		sprintf(alertBuff, "Posture Alert: Hunch back.\n");
		UART_putstring(alertBuff);
	}
	if (poseCode == 2) {
		sprintf(alertBuff, "Posture Alert: Possible pelvic retroversion.\n");
		UART_putstring(alertBuff);
	}
	if (poseCode == 4) {
		sprintf(alertBuff, "Posture Alert: Abnormal lumbar posture.\n");
		UART_putstring(alertBuff);
	}
	if (poseCode == 5) {
		sprintf(alertBuff, "Posture Alert: Body tilted sideways.\n");
		UART_putstring(alertBuff);
	}
}