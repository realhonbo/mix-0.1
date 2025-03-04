#ifndef ICM_42688_HEADER
#define ICM_42688_HEADER


#define IMU_ICM42688_ID            0x47
#define IMU_WHO_AM_I               0x75
#define IMU_REG_BANK_SEL           0x76
#define IMU_FIFO_CONFIG            0x16
#define IMU_ACCEL_CONFIG0          0x50
#define IMU_GYRO_CONFIG0           0x4F
#define IMU_PWR_MGMT0              0x4E

/* accel and gyro range */
#define AFS_16G                    0x00
#define AFS_8G                     0x01
#define AFS_4G                     0x02
#define AFS_2G                     0x03

#define GFS_2000DPS                0x00
#define GFS_1000DPS                0x01
#define GFS_500DPS                 0x02
#define GFS_250DPS                 0x03

/* output rate: default 1000Hz */
#define ODR_8000Hz                 0x03
#define ODR_4000Hz                 0x04
#define ODR_2000Hz                 0x05
#define ODR_1000Hz                 0x06
#define ODR_200Hz                  0x07
#define ODR_100Hz                  0x08
#define ODR_50Hz                   0x09
#define ODR_25Hz                   0x0A
#define ODR_12_5Hz                 0x0B
#define ODR_500Hz                  0x0F

/* accel data registers */
#define IMU_ACCEL_DATA_X1             0x1F
#define IMU_ACCEL_DATA_X0             0x20
#define IMU_ACCEL_DATA_Y1             0x21
#define IMU_ACCEL_DATA_Y0             0x22
#define IMU_ACCEL_DATA_Z1             0x23
#define IMU_ACCEL_DATA_Z0             0x24

/* gyro data registers */
#define IMU_GYRO_DATA_X1              0x25
#define IMU_GYRO_DATA_X0              0x26
#define IMU_GYRO_DATA_Y1              0x27
#define IMU_GYRO_DATA_Y0              0x28
#define IMU_GYRO_DATA_Z1              0x29
#define IMU_GYRO_DATA_Z0              0x2A

#endif