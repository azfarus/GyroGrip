
#include <FaBo9Axis_MPU9250.h>


class Vector {
public:
    float x;float y;float z;

    // Constructor
    Vector(float x_val = 0.0, float y_val = 0.0, float z_val = 0.0)
        : x(x_val), y(y_val), z(z_val) {}

    void operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return;
    }
    float squared_magnitude() const {
        return x * x + y * y + z * z;
    }

    Vector operator+(const Vector& other) const {
        return Vector(x + other.x, y + other.y, z + other.z);
    }
    void operator-=(const Vector& other)  {
        this->x-=other.x;
        this->y-=other.y;
        this->z-=other.z;
        return;
    }
    void set_vals(float x, float y , float z){
        this->x=x;
        this->y=y;
        this->z=z;
        return;
    }

    void print(){
        Serial.printf("%f %f %f\n",x,y,z);
        return;
    }




    //static funtions
    static Vector cross_product(const Vector* v1, const Vector* v2) {
        return Vector(v1->y * v2->z - v1->z * v2->y,
                      v1->z * v2->x - v1->x * v2->z,
                      v1->x * v2->y - v1->y * v2->x);
    }
    static void complementary_filter(Vector *v1 , Vector *v2,float alpha ){
        v1->x=(1.0f - alpha) * v1->x + alpha * v2->x;
        v1->y=(1.0f - alpha) * v1->y + alpha * v2->y;
        v1->z=(1.0f - alpha) * v1->z + alpha * v2->z;
        return;
    }
};


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

class Matrix {
public:
    Vector row1, row2, row3;

    Matrix(){
        row3 = row2 = row1 = Vector();
    }
    
    Matrix inverse() const {
        Matrix inv;

        
        float det = row1.x * (row2.y * row3.z - row3.y * row2.z) -
                    row2.x * (row1.y * row3.z - row3.y * row1.z) +
                    row3.x * (row1.y * row2.z - row2.y * row1.z);

        
        if (det != 0.0) {
            float inv_det = 1.0 / det;

            
            inv.row1.x = (row2.y * row3.z - row3.y * row2.z) * inv_det;
            inv.row1.y = (row3.x * row2.z - row2.x * row3.z) * inv_det;
            inv.row1.z = (row2.x * row3.y - row3.x * row2.y) * inv_det;

            inv.row2.x = (row3.y * row1.z - row1.y * row3.z) * inv_det;
            inv.row2.y = (row1.x * row3.z - row3.x * row1.z) * inv_det;
            inv.row2.z = (row3.x * row1.y - row1.x * row3.y) * inv_det;

            inv.row3.x = (row1.y * row2.z - row2.y * row1.z) * inv_det;
            inv.row3.y = (row2.x * row1.z - row1.x * row2.z) * inv_det;
            inv.row3.z = (row1.x * row2.y - row2.x * row1.y) * inv_det;
        } else {
           
            inv.row1 = Vector(NAN, NAN, NAN);
            inv.row2 = Vector(NAN, NAN, NAN);
            inv.row3 = Vector(NAN, NAN, NAN);
        }

        return inv;
    }

    void print(){
        Serial.printf("%f %f %f\n",row1.x,row1.y,row1.z);
        Serial.printf("%f %f %f\n",row2.x,row2.y,row2.z);
        Serial.printf("%f %f %f\n-------------------------\n",row3.x,row3.y,row3.z);
        return;
    }
};


float invSqrt(float x);
void cross_product_filter(Vector * accel , Vector * Mag , Matrix * output);

