#include "custom_mpu9250.h"





void calibrate( MPU9250_Custom * mpu , mpu_matrix *output , mpu_vector * calib){
    
    mpu_vector accel_loc , mag_loc;
    mpu_vector accel_inp , gyro_inp, mag_inp;

    for(int i = 0 ; i < 1000 ; i++){
        mpu->readAccelXYZVector(&accel_inp,&mag_inp,&gyro_inp);
        accel_loc+=accel_inp;
        mag_loc+=mag_inp;
    }

    accel_loc*=(1.0f / 1000);
    mag_loc*=(1.0f / 1000);

    *calib = accel_loc;

    mpu_matrix static_frame ;

    form_triad(&accel_loc,&mag_loc,&static_frame);



    *output = static_frame.inverse();
}