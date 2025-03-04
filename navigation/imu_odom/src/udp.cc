/**
 * udp transmit for debuggin in vofa remote
 */
#include <sensor_msgs/Imu.h>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>

static sockaddr_in client;


int udp_vofa_init(const char *ip, uint16_t port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == -1)
        return fd;
    client.sin_family = AF_INET;
    client.sin_port = htons(port);
    inet_pton(AF_INET, ip, &client.sin_addr);

    return fd;
}

void udp_vofa_transmit(int fd, char *buffer, size_t size, sensor_msgs::Imu imu)
{
    snprintf(buffer, size, "imu: %6.3f, %6.3f, %6.3f, %6.3f\r\n",
        imu.orientation.w,
        imu.orientation.x,
        imu.orientation.y,
        imu.orientation.z
    );
    sendto(fd, buffer, strlen(buffer), 0, (sockaddr *)&client, sizeof(client));
}