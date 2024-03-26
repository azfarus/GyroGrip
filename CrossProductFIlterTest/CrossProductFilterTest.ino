#include <Wire.h>
#include "CrossProductFIlter.h"



#define OFFSET_CALIB_ITR 2000

Custom_Fabo9axis fabo_9axis;
TaskHandle_t SerialPrintTask;

Vector accel , gyro , mag ,
       accel_filt , gyro_filt , mag_filt,
       gyro_offset; 
float  alpha=0.0005;


float roll , pitch , yaw;
Matrix output , calib_orientation_inverse;
Matrix dcm;
Vector rpy;
uint32_t start, end, looptime;


void calculate_gyro_offset();
void adjust_gyro_offset();
void init();
void apply_complementary_filter();
void SerialPrintTaskFunc(void *parameter);

void init()
{
    calculate_gyro_offset();
    adjust_gyro_offset();
    accel_filt = accel;
    gyro_filt = gyro;
    mag_filt = mag;
}

void SerialPrintTaskFunc(void *parameter)
{
    for (;;)
    {

        Matrix m  = calib_orientation_inverse * output.inverse() ;
        m.row1.print();
        m.row2.print();
        m.row3.print();
        Serial.println();

        delay(50);
    }
}

void calculate_gyro_offset()
{

    for (int i = 0; i < OFFSET_CALIB_ITR; i++)
    {
        fabo_9axis.readAccelXYZVector(&accel, &mag, &gyro);
        gyro_offset = gyro_offset + gyro;
    }
    gyro_offset *= (1.0 / OFFSET_CALIB_ITR);
}

void adjust_gyro_offset()
{
    gyro -= gyro_offset;
}

void apply_complementary_filter()
{
    Vector::complementary_filter(&accel_filt, &accel, alpha);
    Vector::complementary_filter(&gyro_filt, &gyro, alpha);
    Vector::complementary_filter(&gyro_filt, &gyro, alpha);
}






 

void setup()
{

    Serial.begin(115200);
    Serial.println("RESET");
    Serial.println();

    Serial.println("configuring device.");
    // Wire.begin();

    if (fabo_9axis.begin()){   
        Wire.setClock(800000);
        init();
        Serial.println("configured FaBo 9Axis I2C Brick");
    }

    else{
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
    
    calculate_calib_orientation_inverse(&fabo_9axis , &calib_orientation_inverse);
}

void loop()
{

    start = micros();

    fabo_9axis.readAccelXYZVector(&accel,&mag,&gyro);

    adjust_gyro_offset();
    apply_complementary_filter();
    cross_product_filter(&accel , &mag , &output);

    
    //dcm =   output * calib_orientation_inverse ;

   
    

    end = micros();
    looptime = end - start;
}


