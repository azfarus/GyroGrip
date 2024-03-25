/**
 @file read9axis.ino
 @brief This is an Example for the FaBo 9Axis I2C Brick.

   http://fabo.io/202.html

   Released under APACHE LICENSE, VERSION 2.0

   http://www.apache.org/licenses/

 @author FaBo<info@fabo.io>
*/

#include <Wire.h>
#include "CrossProductFIlter.h"
#include <FaBo9Axis_MPU9250.h>


#define OFFSET_CALIB_ITR 2000


FaBo9Axis fabo_9axis;
TaskHandle_t Core1Task;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

float ax_fil, ay_fil, az_fil;
float gx_fil, gy_fil, gz_fil;
float mx_fil, my_fil, mz_fil;


float gx_off, gy_off, gz_off;
float mx_off, my_off, mz_off;

float alpha = 1;

float q0=1, q1 , q2 , q3;
float roll , pitch , yaw;
uint32_t start, end, looptime;


void calcoffsets();
void adjustoffsets();
void Task1code(void *parameter);
void applyFilter();



void setup()
{

    Serial.begin(115200);
    Serial.println("RESET");
    Serial.println();

    Serial.println("configuring device.");
    Wire.setClock(800000);

    if (fabo_9axis.begin()){   
        calcoffsets();
        Serial.println("configured FaBo 9Axis I2C Brick");
    }

    else{
        Serial.println("device error");
        while (1);
    }

    xTaskCreatePinnedToCore(
        Task1code,  /* Function to implement the task */
        "Task1code",  /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        0,          /* Priority of the task */
        &Core1Task, /* Task handle. */
        0);
}

void loop()
{

    start = micros();

    fabo_9axis.readAccelXYZ(&ax, &ay, &az);
    fabo_9axis.readGyroXYZ(&gx, &gy, &gz);
    fabo_9axis.readMagnetXYZ(&mx, &my, &mz);
    adjustoffsets();
    applyFilter();



    

    end = micros();
    looptime = end - start;
}




void calcoffsets(){
    for(int i=0; i< OFFSET_CALIB_ITR ; i++){
        fabo_9axis.readGyroXYZ(&gx, &gy, &gz);
        fabo_9axis.readMagnetXYZ(&mx, &my, &mz);
        
        gx_off+=gx;gy_off+=gy;gz_off+=gz;
        mx_off+=mx;my_off+=my;mz_off+=mz;
    }
    gx_off/=1000;gy_off/=1000;gz_off/=1000;
    mx_off/=1000;my_off/=1000;mz_off/=1000;
}

void adjustoffsets(){
    mx-=mx_off;my-=my_off;mz-=mz_off;
    gx-=gx_off;gy-=gy_off;gz-=gz_off;
    gx*=DEG_TO_RAD;gy*=DEG_TO_RAD;gz*=DEG_TO_RAD;
}

void Task1code(void *parameter)
{
    for (;;)
    {

        Serial.printf("%f %f %f %d\n", roll*RAD_TO_DEG  , pitch*RAD_TO_DEG , yaw*RAD_TO_DEG , looptime);
        delayMicroseconds(1000);
    }
}


void applyFilter(){
    gx_fil = (1.0f - alpha) * gx_fil + alpha * gx;
    gy_fil = (1.0f - alpha) * gy_fil + alpha * gy;
    gz_fil = (1.0f - alpha) * gz_fil + alpha * gz;

    ax_fil = (1.0f - alpha) * ax_fil + alpha * ax;
    ay_fil = (1.0f - alpha) * ay_fil + alpha * ay;
    az_fil = (1.0f - alpha) * az_fil + alpha * az;
    
    mx_fil = (1.0f - alpha) * mx_fil + alpha * mx;
    my_fil = (1.0f - alpha) * my_fil + alpha * my;
    mz_fil = (1.0f - alpha) * mz_fil + alpha * mz;
}