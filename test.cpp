#include<iostream>


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
        printf("%f %f %f\n",x,y,z);
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
            inv.row2.x = (row3.x * row2.z - row2.x * row3.z) * inv_det;
            inv.row3.x = (row2.x * row3.y - row3.x * row2.y) * inv_det;

            inv.row1.y = (row3.y * row1.z - row1.y * row3.z) * inv_det;
            inv.row2.y = (row1.x * row3.z - row3.x * row1.z) * inv_det;
            inv.row3.y = (row3.x * row1.y - row1.x * row3.y) * inv_det;

            inv.row1.z = (row1.y * row2.z - row2.y * row1.z) * inv_det;
            inv.row2.z = (row2.x * row1.z - row1.x * row2.z) * inv_det;
            inv.row3.z = (row1.x * row2.y - row2.x * row1.y) * inv_det;
        } else {
          
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

    void print(){
        printf("%f %f %f\n",row1.x,row1.y,row1.z);
        printf("%f %f %f\n",row2.x,row2.y,row2.z);
        printf("%f %f %f\n-------------------------\n",row3.x,row3.y,row3.z);
        return;
    }
};

int main(){
    Matrix m ;
        m.row1 = Vector(1 , 0 , 2);
        m.row2 = Vector(0 , 1 , 0);
        m.row3 = Vector(0 , 0 , 1);

        Matrix minv = m.inverse();
        Matrix res = m * minv;

        minv.print();
}