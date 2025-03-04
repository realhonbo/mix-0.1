#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "include/multi.h"


int serial_init(serial::Serial &serial, const std::string &port, uint32_t baudrate)
{
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serial.setPort(port);
    serial.setBaudrate(baudrate);
    serial.setTimeout(timeout);
    try {
        serial.open();
        return 0;
    } catch(serial::IOException &except) {
        return -EIO;
    }
}


/**
 * remap uart-rx-buffer to imu and odom data
 *
 * quanternion of odometry uses imu's
 */
int rx_buffer_remap(struct rx_pack buffer, sensor_msgs::Imu &imu, nav_msgs::Odometry &odom)
{
    int i;

    if (buffer.header_a != 0xff || buffer.header_b != 0xff)
        return -EIO;

    if (calculate_crc32((uint8_t *)&buffer.imu, sizeof(buffer.imu) + sizeof(buffer.odom))
        != buffer.crc) {
        return -EIO;
    }

    // update:
    imu.orientation.w = buffer.imu.quanternion[0];
    imu.orientation.x = buffer.imu.quanternion[1];
    imu.orientation.y = buffer.imu.quanternion[2];
    imu.orientation.z = buffer.imu.quanternion[3];

    imu.angular_velocity.x = buffer.imu.gyro_v[0];
    imu.angular_velocity.y = buffer.imu.gyro_v[1];
    imu.angular_velocity.z = buffer.imu.gyro_v[2];

    imu.linear_acceleration.x = buffer.imu.accel_v[0];
    imu.linear_acceleration.y = buffer.imu.accel_v[1];
    imu.linear_acceleration.z = buffer.imu.accel_v[2];

    odom.pose.pose.position.x = buffer.odom.position[0];
    odom.pose.pose.position.y = buffer.odom.position[1];
    odom.pose.pose.position.z = 0;

    odom.twist.twist.linear.x = buffer.odom.twist[0];
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = buffer.odom.twist[1];

    return 0;
}