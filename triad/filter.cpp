#include "filter.h"


void form_triad(mpu_vector * v1 , mpu_vector * v2 , mpu_matrix * output ){
    v2->normalize();

    mpu_vector col1 = v1->cross(*v2);
    mpu_vector col2 = v2->cross(col1);

    col1.normalize();
    col2.normalize();
    
    (output)->coeffRef(0,0) = col1(0);
    (output)->coeffRef(1,0) = col1(1);
    (output)->coeffRef(2,0) = col1(2);

    (output)->coeffRef(0,1)= col2(0);
    (output)->coeffRef(1,1)= col2(1);
    (output)->coeffRef(2,1)= col2(2);

    (output)->coeffRef(0,2)= -(*v1)(0);
    (output)->coeffRef(1,2)= -(*v1)(1);
    (output)->coeffRef(2,2)= -(*v1)(2);

    
}
