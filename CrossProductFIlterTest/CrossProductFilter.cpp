#include "CrossProductFilter.h"

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


void cross_product_filter(Vector * accel , Vector * mag , Matrix * output){
    *accel *= invSqrt(accel->squared_magnitude());
    *mag *= invSqrt(mag->squared_magnitude());

    output->row1 = Vector::cross_product(accel  , mag);
    output->row2 = Vector::cross_product( &output->row1  , mag);
    output->row3 = *mag;
    return;
}