#ifndef _IMU2_H_
#define _IMU2_H_

#include "I2Cdev.h"
#include "imu_config.h"
#include <Wire.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/quaternion.h>

# define M_PI           3.14159265358979323846

using namespace std;

bool initIMU()
{
    Wire.begin();
    bool ret;
    
    accelerometer.initialize();
    ret = accelerometer.testConnection();
    if(!ret)
        return false;

    gyroscope.initialize();
    ret = gyroscope.testConnection();
    if(!ret)
        return false;
  
    magnetometer.initialize();
    ret = magnetometer.testConnection();
    if(!ret)
        return false;

    return true;
}

geometry_msgs__msg__Vector3 readAccelerometer()
{
    geometry_msgs__msg__Vector3 accel;
    int16_t ax, ay, az;
    
    accelerometer.getAcceleration(&ax, &ay, &az);

    accel.x = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.y = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.z = az * (double) ACCEL_SCALE * G_TO_ACCEL;

    return accel;
}

//geometry_msgs::vector3 readGyroscope()
geometry_msgs__msg__Vector3 readGyroscope()
{
    geometry_msgs__msg__Vector3 gyro;
    int16_t gx, gy, gz;

    gyroscope.getRotation(&gx, &gy, &gz);

    gyro.x = gx * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.y = gy * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.z = gz * (double) GYRO_SCALE * DEG_TO_RAD;

    return gyro;
}

geometry_msgs__msg__Vector3 readMagnetometer()
{
    geometry_msgs__msg__Vector3 mag;
    int16_t mx, my, mz;
    
    magnetometer.getHeading(&mx, &my, &mz);
    
    mag.x = mx * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.y = my * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.z = mz * (double) MAG_SCALE * UTESLA_TO_TESLA;
    return mag;
}

#endif
