#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>
#include <MPU9250.h>



#define mpu_vector Eigen::Vector3f
#define mpu_matrix Eigen::Matrix3f


void form_triad(mpu_vector *  v1 , mpu_vector *  v2  , mpu_matrix * output);





