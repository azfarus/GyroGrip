
#include "CrossProductFilter.h"
#include <BluetoothSerial.h>



#define OFFSET_CALIB_ITR 2000
#define DIG_READ_FAST(pin) (*(volatile uint32_t *)(GPIO_IN_REG) & (1 << pin))




void calculate_gyro_offset();
void adjust_gyro_offset();
void init();
void apply_complementary_filter();
void SerialPrintTaskFunc(void* parameter);
void make_angular_vel_tensor();
void output_pin_setup();
void led_blink();



const uint32_t sw1 = 14;
const uint32_t sw2 = 26;
const uint32_t sw3 = 25;
const uint32_t sw4 = 27;
const uint32_t ledpin=2;

MPU9250_Custom my_mpu;
TaskHandle_t SerialPrintTask;
BluetoothSerial SerialBT;

Vector accel, gyro, mag, v_calib,
accel_filt, gyro_filt, mag_filt,
gyro_offset;
float  alpha = 0.05f;


float roll, pitch, yaw;
Matrix output, calib_orientation_inverse, angular_vel_tensor;
Matrix dcm, rot_mat;
Vector rpy;
uint32_t start, end, looptime;


void setup() {

    output_pin_setup();

    led_blink();

    esp_sleep_enable_ext0_wakeup(GPIO_NUM_14, 1);
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

    calculate_calib_orientation_inverse(&my_mpu, &calib_orientation_inverse, &v_calib);
    xTaskCreatePinnedToCore(
        SerialPrintTaskFunc,  /* Function to implement the task */
        "SerialPrintTask",  /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        0,          /* Priority of the task */
        &SerialPrintTask, /* Task handle. */
        0);

    rot_mat.row1 = Vector(1, 0, 0);
    rot_mat.row2 = Vector(0, 1, 0);
    rot_mat.row3 = Vector(0, 0, 1);


}

void loop() {

    start = micros();

    my_mpu.readAccelXYZVector(&accel, &mag, &gyro);

    adjust_gyro_offset();
    apply_complementary_filter();

    make_angular_vel_tensor();

    cross_product_filter(&accel_filt, &mag_filt, &output);


    end = micros();

    rot_mat = rot_mat + angular_vel_tensor * ((end - start) * 1e-6);

    Matrix m = (calib_orientation_inverse * output).inverse();
    rot_mat = rot_mat * (0.995f) + (m) * (0.005f);

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

        bool isHighsw1 = DIG_READ_FAST(sw1);
        bool isHighsw2 = DIG_READ_FAST(sw2);
        bool isHighsw3 = DIG_READ_FAST(sw3);
        bool isHighsw4 = DIG_READ_FAST(sw4);


        Vector v = vec_into_mat(&v_calib, &rot_mat);
        SerialBT.printf("%f %f %d %d %d %d\n", v.y, v.z, isHighsw1, isHighsw2, isHighsw3, isHighsw4);

        delayMicroseconds(20000);

        if(isHighsw1 && isHighsw2 && isHighsw3 && isHighsw4 ) {
           led_blink();
           esp_deep_sleep_start();
        }
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

void make_angular_vel_tensor() {
    angular_vel_tensor.row1.x = 0.0f;
    angular_vel_tensor.row1.y = gyro_filt.z;
    angular_vel_tensor.row1.z = -gyro_filt.y;

    angular_vel_tensor.row2.x = -gyro_filt.z;
    angular_vel_tensor.row2.y = 0.0f;
    angular_vel_tensor.row2.z = gyro_filt.x;

    angular_vel_tensor.row3.x = gyro_filt.y;
    angular_vel_tensor.row3.y = -gyro_filt.x;
    angular_vel_tensor.row3.z = 0.0f;
}

void output_pin_setup() {
    pinMode(ledpin,OUTPUT);
    pinMode(sw1, INPUT_PULLDOWN);
    pinMode(sw2, INPUT_PULLDOWN);
    pinMode(sw3, INPUT_PULLDOWN);
    pinMode(sw4, INPUT_PULLDOWN);
}

void led_blink(){
    for(int i = 0 ; i < 10;i++){
        digitalWrite(ledpin,!digitalRead(ledpin));
        delay(300);
    }
}