#pragma once

// crc calibration
uint32_t calculate_crc32(const uint8_t *, int);


// udp: vofa debuggin
#define UDP_BUF_SIZE        64

int  udp_vofa_init(const char *, uint16_t);
void udp_vofa_transmit(int, char *, size_t, sensor_msgs::Imu);


// serial: connect stm32
#define RX_DATA_BYTES       (40 + 16)
#define BUF_HEAD_BYTES       2
#define CALIB_BYTES          4
#define RX_BUF_SIZE		    (BUF_HEAD_BYTES + RX_DATA_BYTES + CALIB_BYTES)

struct rx_pack {
    uint8_t header_a;
    uint8_t header_b;
    struct {
        float quanternion[4];
        float gyro_v[3];
        float accel_v[3];
    } imu;
    struct {
        float position[2];
        float twist[2];
    } odom;
    uint32_t crc;
} __attribute__((packed));

struct tx_pack {
    uint8_t header_a;
    uint8_t header_b;
    struct {
        float l_vel;
        float r_vel;
    } cmd;
    uint32_t crc;
} __attribute((packed));

int serial_init(serial::Serial &, const std::string &, uint32_t);
int rx_buffer_remap(struct rx_pack, sensor_msgs::Imu &, nav_msgs::Odometry &);
