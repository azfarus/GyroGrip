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