#ifndef ImuPublisher_h
#define ImuPublisher_h
#include <Arduino.h>
#include <Timer.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <Zumo32U4IMU.h>
#include <Wire.h>

/*
 * https://atadiat.com/en/e-ros-imu-and-arduino-how-to-send-to-ros/
 */
class ImuPublisher
{
private:
    Timer timer;
    Zumo32U4IMU imu;

    DynamicJsonDocument outputDocument = DynamicJsonDocument(200);

    // http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
    sensor_msgs::Imu imu_msg;
    // ros::Publisher pub_imu = ros::Publisher("imu", &imu_msg);

public:
    ImuPublisher()
    {
    }

    void setup()
    {
        outputDocument["topic"] = "imu";
        outputDocument["la"]["x"] = (int16_t)0;
        outputDocument["la"]["y"] = (int16_t)0;
        outputDocument["la"]["z"] = (int16_t)0;
        outputDocument["av"]["x"] = (int16_t)0;
        outputDocument["av"]["y"] = (int16_t)0;
        outputDocument["av"]["z"] = (int16_t)0;
        outputDocument["o"]["x"] = (int16_t)0;
        outputDocument["o"]["y"] = (int16_t)0;
        outputDocument["o"]["z"] = (int16_t)0;
        outputDocument["o"]["w"] = (int16_t)0;

        // TODO: Set the covariances to something reasonable
        // imu_msg.orientation_covariance = {-1, 0, 0, 0, -1, 0, 0, 0, 0};
        // imu_msg.angular_velocity_covariance = {-1, 0, 0, 0, -1, 0, 0, 0, 0};
        // imu_msg.linear_acceleration_covariance = {-1, 0, 0, 0, -1, 0, 0, 0, 0};

        outputDocument.shrinkToFit();

        Wire.begin();
        imu.init();
        imu.enableDefault();
    }

    bool updateAcc()
    {
        imu.readAcc();
        if (imu.accDataReady())
        {
            outputDocument["la"]["x"] = imu.a.x;
            outputDocument["la"]["y"] = imu.a.y;
            outputDocument["la"]["z"] = imu.a.z;
            return true;
        }
        return false;
    }

    bool updateGyro()
    {
        imu.readGyro();
        if (imu.gyroDataReady())
        {
            outputDocument["av"]["x"] = imu.g.x;
            outputDocument["av"]["y"] = imu.g.y;
            outputDocument["av"]["z"] = imu.g.z;
            return true;
        }
        return false;
    }

    // From GithubCopilot. I have no idea what this function does.
    // TODO: Find out what this function does
    int16_t calculateQuaternionWValueFromMagnetometerData(int16_t x, int16_t y, int16_t z)
    {
        float w = sqrt(1.0 + x + y + z) / 2.0;
        return w;
    }

    bool updateMag()
    {
        imu.readMag();
        if (imu.magDataReady())
        {
            outputDocument["o"]["x"] = imu.m.x;
            outputDocument["o"]["y"] = imu.m.y;
            outputDocument["o"]["z"] = imu.m.z;
            outputDocument["o"]["w"] = calculateQuaternionWValueFromMagnetometerData(imu.m.x, imu.m.y, imu.m.z);
            return true;
        }
        return false;
    }

    void loop()
    {
        if (timer.loopWait(1))
        {
            // TODO: Research best-practice for publishing this message
            // with partially updated values.
            // Example:
            // If only the accelerometer has new values, should old readings
            // from the gyro and magnetometer be included?
            bool updatedAcc = updateAcc();
            bool updatedGyro = updateGyro();
            bool updatedMag = updateMag();

            if (updatedAcc || updatedGyro || updatedMag)
            {
                serializeJson(outputDocument, DATA_SERIAL_CLASS);
            }
        }
    }
};

#endif
