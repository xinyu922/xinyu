#ifndef AHRS_h
#define AHRS_h

// Variable declaration
extern int instability_fix;
extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

// Function declarations
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif

