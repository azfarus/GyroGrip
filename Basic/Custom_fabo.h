#include <FaBo9Axis_MPU9250.h>
#include "Vector.h"

class Custom_Fabo9axis : public FaBo9Axis{

    private:
    float   ax,ay,az,
            mx,my,mz,
            gx,gy,gz; 
    public: 
    void readAccelXYZVector(Vector * accel , Vector * mag , Vector * gyro){
        
        readAccelXYZ ( &ax , &ay , &az );
        readMagnetXYZ( &mx , &my , &mz );
        readGyroXYZ  ( &gx , &gy , &gz );

        accel->set_vals(ax,ay,az);
        mag->set_vals(mx,my,mz);
        gyro->set_vals(gx,gy,gz);
        return;
    }
};
