#include <MPU9250_Master_I2C.h>
#include <iostream>
#include <wiringPi.h>
#include <errmsg.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Publisher imu_pub_;
sensor_msgs::Imu data;
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
    ros::init(argc, argv, "imu_pub");
    ros::NodeHandle n("");

    imu_pub_ = n.advertise<sensor_msgs::Imu>("imu",50);

    setup();

    static float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

    double hz=100;
    ros::Rate r(hz);
    while(ros::ok()){
        if(imu.checkNewData()){
            imu.readAccelerometer(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
            imu.readGyrometer(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z);
        }
        r.sleep();
    }
}