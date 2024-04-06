
#include <Arduino.h>
#include<HardwareSerial.h>
#include <MPU9250.h>

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
    void operator+=(const Vector& other)  {
        this->x+=other.x;
        this->y+=other.y;
        this->z+=other.z;
        return;
    }
    void set_vals(float x, float y , float z){
        this->x=x;
        this->y=y;
        this->z=z;
        return;
    }

    void print(){
        Serial.printf("%f %f %f ",x,y,z);
        return;
    }
    float dot(Vector v){
        return this->x*v.x + this->y*v.y + this->z*v.z;
    }

    Vector operator*(float val){
        return Vector(this->x*val,this->y*val,this->z*val);
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



class MPU9250_Custom {
    private:

        MPU9250 mpu;

    public: 

    bool setup(uint8_t addr){
        return mpu.setup(addr);
    }



    void readAccelXYZVector(Vector * accel , Vector * mag , Vector * gyro){
        if (!mpu.available()) return;
        mpu.update_accel_gyro();
        mpu.update_mag();

        accel->set_vals(-mpu.getAccX(),-mpu.getAccY(),-mpu.getAccZ());
        mag->set_vals(mpu.getMagY(),mpu.getMagX(),-mpu.getMagZ());

        gyro->set_vals( mpu.getGyroX()*DEG_TO_RAD ,mpu.getGyroY()*DEG_TO_RAD ,mpu.getGyroZ()*DEG_TO_RAD);

        // gyro->print();
        // Serial.println();
        
        return;
    }
};

class Matrix {
public:
    Vector row1, row2, row3;

    Matrix(){
        row3 = row2 = row1 = Vector();
    }

    void operator=(const Matrix& other){
        this->row1 = other.row1;
        this->row2 = other.row2;
        this->row3 = other.row3;
    }
    
    Matrix inverse() const {
        Matrix inv;

        
        float det = row1.x * (row2.y * row3.z - row3.y * row2.z) -
                    row2.x * (row1.y * row3.z - row3.y * row1.z) +
                    row3.x * (row1.y * row2.z - row2.y * row1.z);

        
        if (det != 0.0) {
            float inv_det = 1.0 / det;

            
            inv.row1.x = (row2.y * row3.z - row3.y * row2.z) * inv_det;
            inv.row2.x = (row3.x * row2.z - row2.x * row3.z) * inv_det;
            inv.row3.x = (row2.x * row3.y - row3.x * row2.y) * inv_det;

            inv.row1.y = (row3.y * row1.z - row1.y * row3.z) * inv_det;
            inv.row2.y = (row1.x * row3.z - row3.x * row1.z) * inv_det;
            inv.row3.y = (row3.x * row1.y - row1.x * row3.y) * inv_det;

            inv.row1.z = (row1.y * row2.z - row2.y * row1.z) * inv_det;
            inv.row2.z = (row2.x * row1.z - row1.x * row2.z) * inv_det;
            inv.row3.z = (row1.x * row2.y - row2.x * row1.y) * inv_det;
        } else {
           
            inv.row1 = Vector(NAN, NAN, NAN);
            inv.row2 = Vector(NAN, NAN, NAN);
            inv.row3 = Vector(NAN, NAN, NAN);
        }

        return inv;
    }



    Matrix operator*(const Matrix& other) const {
        Matrix result;
        result.row1.x = row1.x * other.row1.x + row1.y * other.row2.x + row1.z * other.row3.x;
        result.row1.y = row1.x * other.row1.y + row1.y * other.row2.y + row1.z * other.row3.y;
        result.row1.z = row1.x * other.row1.z + row1.y * other.row2.z + row1.z * other.row3.z;

        result.row2.x = row2.x * other.row1.x + row2.y * other.row2.x + row2.z * other.row3.x;
        result.row2.y = row2.x * other.row1.y + row2.y * other.row2.y + row2.z * other.row3.y;
        result.row2.z = row2.x * other.row1.z + row2.y * other.row2.z + row2.z * other.row3.z;

        result.row3.x = row3.x * other.row1.x + row3.y * other.row2.x + row3.z * other.row3.x;
        result.row3.y = row3.x * other.row1.y + row3.y * other.row2.y + row3.z * other.row3.y;
        result.row3.z = row3.x * other.row1.z + row3.y * other.row2.z + row3.z * other.row3.z;

        return result;
    }

    Matrix operator+(const Matrix& other) const {
        Matrix result;

        result.row1 = this->row1 + other.row1;
        result.row2 = this->row2 + other.row2;
        result.row3 = this->row3 + other.row3;

        return result;
    }
    Matrix operator*(float val) {
        Matrix result;

        result.row1 = this->row1 * val;
        result.row2 = this->row2 * val;
        result.row3 = this->row3 * val;

        return result;
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
void calculate_calib_orientation_inverse(MPU9250_Custom * mpu , Matrix *output , Vector * calib);
void calculate_euler_from_dcm(Matrix * dcm , Vector * rpy);
Vector vec_into_mat(Vector * v , Matrix * m );



