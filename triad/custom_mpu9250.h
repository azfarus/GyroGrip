#include "filter.h"

class MPU9250_Custom {
    private:

        MPU9250 mpu;

    public: 

    bool setup(uint8_t addr){
        return mpu.setup(addr);
    }



    void readAccelXYZVector(mpu_vector * accel , mpu_vector * mag , mpu_vector * gyro){
        if (!mpu.available()) return;
        mpu.update_accel_gyro();
        mpu.update_mag();
    
        //accel->set_vals(-mpu.getAccX(),-mpu.getAccY(),-mpu.getAccZ());

        (*accel)(0) = -mpu.getAccX();
        (*accel)(1) = -mpu.getAccY();
        (*accel)(2) = -mpu.getAccZ();
        
        //mag->set_vals(mpu.getMagY(),mpu.getMagX(),-mpu.getMagZ());

        (*mag)(0) =  mpu.getMagY();
        (*mag)(1) =  mpu.getMagX();
        (*mag)(2) = -mpu.getMagZ();

        //gyro->set_vals( mpu.getGyroX()*DEG_TO_RAD ,mpu.getGyroY()*DEG_TO_RAD ,mpu.getGyroZ()*DEG_TO_RAD);

        (*gyro)(0) =  mpu.getGyroX()*DEG_TO_RAD;
        (*gyro)(1) =  mpu.getGyroY()*DEG_TO_RAD;
        (*gyro)(2) = -mpu.getGyroZ()*DEG_TO_RAD;
        
        
        
        return;
    }
};


void calibrate( MPU9250_Custom * mpu , mpu_matrix *output , mpu_vector * calib);