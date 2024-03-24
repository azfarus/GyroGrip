#include <math.h>

#define beta 1.0f
#define sampleFreq 1428.0f



float invSqrt(float x);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz , float &q0 , float &q1 , float &q2 , float &q3);
void quaternion_to_euler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw);