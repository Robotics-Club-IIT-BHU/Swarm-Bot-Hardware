#include <MPU9250_Master_I2C.h>
#include <iostream>
#include <wiringPi.h>
#include <errmsg.h>
#include <ros/ros.h>

static const MPUIMU::Gscale_t GSCALE     = MPUIMU::GFS_250DPS;
static const MPUIMU::Ascale_t ASCALE     = MPUIMU::AFS_2G;
static const MPU9250::Mscale_t MSCALE    = MPU2950::MFS_16BITS;
static const MPU9250::Mmode_t MMODE      = MPU9250::M_100Hz;
static const uint8_t SAMPLE_RATE_DIVISOR = 0x04;

static const uint8_t intPin = 0;

static bool gotNewData;
static void myinthandler(){
    gotNewData=true;
}

static MPU9250_Master_I2C imu(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISIOR);

void setup()
{
    wiringPiSetup();

    switch(imu.begin()){

        case MPUIMU::ERROR_IMU_ID:
            errmsg("Bad IMU device ID");
            break;
        case MPUIMU::ERROR_MAG_ID:
            errmsg("Bad magnetometer device ID");
            break;
        case MPUIMU::ERROR_SELFTEST:
            break;
        default:
            printf("MPU6050 online!\n");
            break;
    }
    wiringPiISR(intPin, INT_EDGE_RISING, &myinthandler);
}

int main(){
    static float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

    double hz=100;
    if(true) {

        gotNewData = false;    
        if (imu.checkNewData())  { 

            imu.readAccelerometer(ax, ay, az);
            imu.readGyrometer(gx, gy, gz);
            imu.readMagnetometer(mx, my, mz);
            temperature = imu.readTemperature();
        }
    }

    
    static uint32_t msec_prev;
    uint32_t msec_curr = millis();

    if (msec_curr-msec_prev > 1000.0/hz) {

        msec_prev = msec_curr;

        printf("\n");

        printf("ax = %d  ay = %d  az = %d mg\n", (int)(1000*ax), (int)(1000*ay), (int)(1000*az));
        printf("gx = %+2.2f  gy = %+2.2f  gz = %+2.2f deg/s\n", gx, gy, gz);
        printf("mx = %d  my = %d  mz = %d mG\n", (int)mx, (int)my, (int)mz);

        // Print temperature in degrees Centigrade      
        printf("Gyro temperature is %+1.1f degrees C\n", temperature);  
    }
}