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
    *accel *= (1.0f)/sqrtf(accel->squared_magnitude());
    *mag *= (1.0f)/sqrtf(mag->squared_magnitude());

    output->row2 = Vector::cross_product(accel  , mag);
    output->row1 = Vector::cross_product( &output->row2  , accel);
    *accel *= -1.0f;
    output->row3 = (*accel);
    return;
}

void calculate_calib_orientation_inverse(Custom_Fabo9axis * mpu , Matrix *output){
    Vector accel_loc , mag_loc;
    Vector accel_inp , gyro_inp, mag_inp;

    for(int i = 0 ; i < 1000 ; i++){
        mpu->readAccelXYZVector(&accel_inp,&mag_inp,&gyro_inp);
        accel_loc+=accel_inp;
        mag_loc+=mag_inp;
    }

    accel_loc*=(1.0f / 1000);
    mag_loc*=(1.0f / 1000);

    Matrix static_frame ;

    cross_product_filter(&accel_loc,&mag_loc,&static_frame);

    *output = static_frame.inverse();

        
}

void calculate_euler_from_dcm(Matrix * dcm , Vector * rpy){
    float a = dcm->row1.x;
    float b = dcm->row2.x;
    float c = dcm->row3.x;
    float d = dcm->row3.y;
    float e = dcm->row3.z;


    rpy->x = atan2f(d,e);
    rpy->y = -asinf(c);
    rpy->z = atan2f(b,a);
    return;
}