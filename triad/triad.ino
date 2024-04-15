#include "custom_mpu9250.h"
#include <BluetoothSerial.h>
#define OFFSET_CALIB_ITR 5000


void calculate_gyro_offset();
void adjust_gyro_offset();
void init();
void apply_complementary_filter();
void SerialPrintTaskFunc(void* parameter);
void make_angular_vel_tensor();


MPU9250_Custom my_mpu;
TaskHandle_t SerialPrintTask;
BluetoothSerial SerialBT;

mpu_vector accel, gyro, mag, v_calib,
accel_filt, gyro_filt, mag_filt,
gyro_offset;
float  alpha = 1.0f;



mpu_matrix output, calib_orientation_inverse , angular_vel_tensor,
           rot_mat;
uint32_t start, end, looptime;

void setup(){


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

    calibrate(&my_mpu, &calib_orientation_inverse , &v_calib);
    
    
    xTaskCreatePinnedToCore(
        SerialPrintTaskFunc,  /* Function to implement the task */
        "SerialPrintTask",  /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        0,          /* Priority of the task */
        &SerialPrintTask, /* Task handle. */
        0);
    
    rot_mat.setIdentity();

}
void loop(){
    start = micros();

    my_mpu.readAccelXYZVector(&accel, &mag, &gyro);

    adjust_gyro_offset();
    apply_complementary_filter();

    make_angular_vel_tensor();

    form_triad(&accel_filt, &mag_filt, &output);


    end = micros();

    rot_mat =  ( angular_vel_tensor * ((end - start)*1e-6) ) * rot_mat;

    mpu_matrix m = (output * calib_orientation_inverse ).transpose();
    rot_mat = rot_mat * (0.995f) + (m) *(0.005f);

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



        mpu_vector v =rot_mat * v_calib ;        

        Serial.printf("%f %f\n", v(2), v(1));
        
        delayMicroseconds(20000);
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
    gyro_filt = (1.0f - alpha) * gyro_filt + alpha * gyro;
    accel_filt = (1.0f - alpha) * accel_filt + alpha * accel;
    mag_filt = (1.0f - alpha) * mag_filt + alpha * mag;
}

void make_angular_vel_tensor(){
    angular_vel_tensor.coeffRef(0,0) =  1.0f;
    angular_vel_tensor.coeffRef(0,1) = -gyro_filt(2);
    angular_vel_tensor.coeffRef(0,2) =  gyro_filt(1);

    angular_vel_tensor.coeffRef(1,0) =  gyro_filt(2);
    angular_vel_tensor.coeffRef(1,1) =  1.0f;
    angular_vel_tensor.coeffRef(1,2) = -gyro_filt(0);

    angular_vel_tensor.coeffRef(2,0) = -gyro_filt(1);
    angular_vel_tensor.coeffRef(2,1) =  gyro_filt(0);
    angular_vel_tensor.coeffRef(2,2) =  1.0f;
}