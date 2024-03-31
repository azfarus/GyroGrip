
#include "CrossProductFilter.h"
#include <BluetoothSerial.h>



#define OFFSET_CALIB_ITR 2000



void calculate_gyro_offset();
void adjust_gyro_offset();
void init();
void apply_complementary_filter();
void SerialPrintTaskFunc(void* parameter);


MPU9250_Custom my_mpu;
TaskHandle_t SerialPrintTask;
BluetoothSerial SerialBT;

Vector accel, gyro, mag,
accel_filt, gyro_filt, mag_filt,
gyro_offset;
float  alpha = 0.25f;


float roll, pitch, yaw;
Matrix output, calib_orientation_inverse;
Matrix dcm;
Vector rpy;
uint32_t start, end, looptime;


void setup() {

    SerialBT.begin("ESP32test");
    delay(1000);

    Serial.begin(115200);
    Serial.println("RESET");
    Serial.println();

    Serial.println("configuring device.");

    Wire.begin();
    Wire.setClock(800000);

    if (my_mpu.setup(0x68)) {
        init();
        Serial.println("Configured MPU9250");
    }
    else {
        Serial.println("device error");
        while (1);
    }

    xTaskCreatePinnedToCore(
        SerialPrintTaskFunc,  /* Function to implement the task */
        "SerialPrintTask",  /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        0,          /* Priority of the task */
        &SerialPrintTask, /* Task handle. */
        0);
        calculate_calib_orientation_inverse(&my_mpu, &calib_orientation_inverse);
}

void loop() {

    start = micros();

    my_mpu.readAccelXYZVector(&accel, &mag, &gyro);

    adjust_gyro_offset();
    apply_complementary_filter();
    cross_product_filter(&accel_filt, &mag_filt, &output);


    end = micros();
    looptime = end - start;

}



void init() {
    
    calculate_gyro_offset();
    adjust_gyro_offset();
    accel_filt = accel;
    gyro_filt = gyro;
    mag_filt = mag;
    
}

void SerialPrintTaskFunc(void* parameter) {
    for (;;) {

        Matrix m = calib_orientation_inverse * output;
        m = m.inverse();
        Vector v = Vector(0, 0, 1);
        v = vec_into_mat(&v, &m);
        SerialBT.printf("%f %f\n", v.x, v.y);
        
        delayMicroseconds(50000);
    }
}

void calculate_gyro_offset() {

    for (int i = 0; i < OFFSET_CALIB_ITR; i++) {
        my_mpu.readAccelXYZVector(&accel, &mag, &gyro);
        gyro_offset = gyro_offset + gyro;
    }
    gyro_offset *= (1.0 / OFFSET_CALIB_ITR);
}

void adjust_gyro_offset() {
    gyro -= gyro_offset;
}

void apply_complementary_filter() {
    Vector::complementary_filter(&accel_filt, &accel, alpha);
    Vector::complementary_filter(&mag_filt, &mag, alpha);
    Vector::complementary_filter(&gyro_filt, &gyro, alpha);
}




