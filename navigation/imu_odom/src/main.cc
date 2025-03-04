/**
 * receive imu[quanternion] and odometry data from stm32f1
 *  [stm32: uart1]==========[orangepi3B: uart7]
 * subscribe </cmd_vel> and publish </imu> and </odometry>
 *
 *
 **Receive:
 *  [orangepi] <--------- [stm32]
 *  format:
 *      header             data             crc
 *    0xff  0xff  { odom-data  imu-data }  32-bit
 *
 *  imu-data [40 bytes]: float quat[4], float angular[3], float accel[3]
 *  odom-data[16 bytes]: float position[2], float twist[2]
 *
 *
 **Transmit:
 *  [orangepi] ---------> [stm32]
 *  format:
 *        header            data             crc
 *     0xee  0xee  { left_vel  right_vel }  32-bit
 *
 *
 * @author Honbo He
 */
#include <ros/ros.h>
#include <unistd.h>
#include <cstring>
// imu odom
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
// communication
#include <serial/serial.h>

#include "include/multi.h"

static struct tx_pack tx_stm32;
static struct rx_pack rx_stm32;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    const float L = 0.18;
    float v, w;

    v = msg->linear.x;
    w = msg->angular.z;

    tx_stm32.cmd.l_vel = v + w*L/2;
    tx_stm32.cmd.r_vel = v - w*L/2;

    tx_stm32.header_a = 0xee;
    tx_stm32.header_b = 0xee;
    tx_stm32.crc = calculate_crc32((uint8_t *)&tx_stm32.cmd, sizeof(tx_stm32.cmd));
}

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "imu_odom");

    // dual-publisher
    ros::NodeHandle node_handle;

    ros::Publisher imu_pub  = node_handle.advertise<sensor_msgs::Imu>("imu", 50);
    ros::Publisher odom_pub = node_handle.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber teleop_sub = node_handle.subscribe("/cmd_vel", 10, cmd_vel_callback);

    // topics msg data
    sensor_msgs::Imu imu;
    nav_msgs::Odometry odom;

    uint8_t  rx_buffer[RX_BUF_SIZE];
	char udp_tx_buffer[UDP_BUF_SIZE];

    // broadcast transform between coordinate frames
    tf::TransformBroadcaster bc;
    tf::Transform tf_;

    // uart-7 serial-bus init
    serial::Serial sbus;
    if (serial_init(sbus, "/dev/ttyS7", 115200)) {
        ROS_ERROR_STREAM("failed in open serial-port" << sbus.getPort());
        return -EIO;
    }
    ROS_INFO("%s: connected to stm32", sbus.getPort().c_str());

    // udp init for vofa debugger
    int sock_fd = udp_vofa_init("192.168.0.3", 11451);

    ros::Rate loop(100);
    int i = 0;
    while (ros::ok()) {
        ros::spinOnce();

        // tx cmd_vel
        sbus.write((uint8_t *)&tx_stm32, sizeof(tx_stm32));
        printf("left: %2.3f right: %2.3f\n", tx_stm32.cmd.l_vel, tx_stm32.cmd.r_vel);

        // rx odom data
        if (sbus.available()) {
            ROS_INFO_ONCE("receiving data...");
            int ava = sbus.available();
            sbus.read((uint8_t *)&rx_stm32, ava);

            if (rx_buffer_remap(rx_stm32, imu, odom)) {
                //ROS_WARN_STREAM("failed in calibration of crc32, size " << ava);
            }
        }

        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "base_link";
        imu_pub.publish(imu);

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.orientation = imu.orientation;
        odom_pub.publish(odom);

        // publish tf of odom to base_link
        tf_.setOrigin( { 0, 0, 0 } );
        tf_.setRotation(tf::Quaternion(
                imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w
        ));
        bc.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "odom", "base_link"));
            // fixed devices
        tf_.setIdentity();
        bc.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "base_link", "laser"));
        bc.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "base_link", "imu"));

        udp_vofa_transmit(sock_fd, udp_tx_buffer, sizeof(udp_tx_buffer), imu);

        loop.sleep();
    }

    sbus.close();
	close(sock_fd);
    return 0;
}
