#ifndef ImuPublisher_h
#define ImuPublisher_h
#include <Arduino.h>
#include <Timer.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <Zumo32U4IMU.h>

/*
 * https://atadiat.com/en/e-ros-imu-and-arduino-how-to-send-to-ros/
 */
class ImuPublisher
{
private:
    Timer timer;
    Zumo32U4IMU imu;

    String frameid = "/imu";

    // http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
    sensor_msgs::Imu imu_msg;
    ros::Publisher pub_imu = ros::Publisher("imu", &imu_msg);

public:
    void setup(ros::NodeHandle &nh)
    {
        nh.advertise(pub_imu);

        imu_msg.header.frame_id = frameid.c_str();

        // TODO: Set the covariances to something reasonable
        // imu_msg.orientation_covariance = {-1, 0, 0, 0, -1, 0, 0, 0, 0};
        // imu_msg.angular_velocity_covariance = {-1, 0, 0, 0, -1, 0, 0, 0, 0};
        // imu_msg.linear_acceleration_covariance = {-1, 0, 0, 0, -1, 0, 0, 0, 0};
    }

    bool updateAcc()
    {
        if (imu.accDataReady())
        {
            imu.readAcc();
            imu_msg.linear_acceleration.x = imu.a.x;
            imu_msg.linear_acceleration.y = imu.a.y;
            imu_msg.linear_acceleration.z = imu.a.z;
            return true;
        }
        return false;
    }

    bool updateGyro()
    {
        if (imu.gyroDataReady())
        {
            imu.readGyro();
            imu_msg.angular_velocity.x = imu.g.x;
            imu_msg.angular_velocity.y = imu.g.y;
            imu_msg.angular_velocity.z = imu.g.z;
            return true;
        }
        return false;
    }

    // From GithubCopilot. I have no idea what this function does.
    // TODO: Find out what this function does
    float calculateQuaternionWValueFromMagnetometerData(float x, float y, float z)
    {
        float w = sqrt(1.0 + x + y + z) / 2.0;
        return w;
    }

    bool updateMag()
    {
        if (imu.magDataReady())
        {
            imu.readMag();
            imu_msg.orientation.x = imu.m.x;
            imu_msg.orientation.y = imu.m.y;
            imu_msg.orientation.z = imu.m.z;
            imu_msg.orientation.w = calculateQuaternionWValueFromMagnetometerData(imu.m.x, imu.m.y, imu.m.z);
            return true;
        }
        return false;
    }

    void loop(ros::NodeHandle &nh)
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
                imu_msg.header.stamp = nh.now();
                pub_imu.publish(&imu_msg);
            }
        }
    }
};

#endif
