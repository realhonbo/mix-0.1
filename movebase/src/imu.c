/**
 * @file imu.c
 * @author Honbo (hehongbo918@gmail.com)
 * @version 1.0
 * @copyright Copyright (c) 2025
 *
 * use ICM42688
 */
#include <stm32f1xx_hal.h>
#include <rtthread.h>
#include <qfplib-m3.h>
#include "include/multi.h"
#include "include/icm42688.h"

// Soft NSS
#define IMU_SPI_CS(N)   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, N)

// 收发缓冲
#define IMU_READ_BUF_SIZE   12
static uint8_t *imu_tx, *imu_rx;

// gyro and accel scale factor
static float gscale;
static float ascale;


static void scale_factor_set(uint8_t gyro, uint8_t acc)
{
    switch (gyro) {
        case GFS_2000DPS: gscale = qfp_fdiv(2000, 32768); break;
        case GFS_1000DPS: gscale = qfp_fdiv(1000, 32768); break;
        case GFS_500DPS:  gscale = qfp_fdiv( 500, 32768); break;
        case GFS_250DPS:  gscale = qfp_fdiv( 250, 32768); break;
    }
    switch (acc) {
        case AFS_16G: ascale = qfp_fdiv(16000, 32768); break;
        case AFS_8G:  ascale = qfp_fdiv( 8000, 32768); break;
        case AFS_4G:  ascale = qfp_fdiv( 4000, 32768); break;
        case AFS_2G:  ascale = qfp_fdiv( 2000, 32768); break;
    }
}


/** SPI2 Init:
 *      CS    > PB12
 *      SCLK  > PB13
 *      MISO  > PB14
 *      MOSI  > PB15
 */
static SPI_HandleTypeDef hspi2;
static void spi_init(void)
{
    GPIO_InitTypeDef IO_Init;

    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
// hardware
    // sck + mosi
    IO_Init.Pin = GPIO_PIN_13 | GPIO_PIN_15;
    IO_Init.Mode = GPIO_MODE_AF_PP;
    IO_Init.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &IO_Init);
    // miso
    IO_Init.Pin = GPIO_PIN_14;
    IO_Init.Mode = GPIO_MODE_INPUT;
    IO_Init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &IO_Init);
    // cs
    IO_Init.Pin = GPIO_PIN_12;
    IO_Init.Pull = GPIO_PULLUP;
    IO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    IO_Init.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &IO_Init);
    IMU_SPI_CS(1);
// software
    // hal
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&hspi2);
}

/**
 * 读写 IMU 寄存器
 *
 ** 最高位为读写位: 读 1, 写 0
 *
 *! ICM-42688 支持突发读, 但不可突发写
 */
static void icm42688_write_register(uint8_t reg, uint8_t val)
{
    imu_tx[0] = reg & 0x7f;
    imu_tx[1] = val;

    IMU_SPI_CS(0);
    HAL_SPI_Transmit(&hspi2, imu_tx, 2, 0xff);
    IMU_SPI_CS(1);
}

static void icm42688_read_register(uint8_t reg, uint8_t *val)
{
    reg |= 0x80;
    imu_tx[0] = 0xff;

    IMU_SPI_CS(0);
    // write register address for reading
    HAL_SPI_Transmit(&hspi2, &reg, 1, 0xff);
    // read data
    HAL_SPI_TransmitReceive(&hspi2, imu_tx, imu_rx, 1, 0xff);
    IMU_SPI_CS(1);

    *val = imu_rx[0];
}

static int icm42688_burst_read(uint8_t reg, uint8_t *buf, int len)
{
    int ret;

    if (len > IMU_READ_BUF_SIZE)
        return -RT_EFULL;

    reg |= 0x80;
    rt_memset(imu_tx, 0, len);

    IMU_SPI_CS(0);
    // send register address
    HAL_SPI_Transmit(&hspi2, &reg, 1, 0xff);
    // read
    ret = HAL_SPI_TransmitReceive(&hspi2, imu_tx, imu_rx, len, 0xff);
    IMU_SPI_CS(1);

    if (ret != HAL_OK)
        return -RT_ERROR;

    rt_memcpy(buf, imu_rx, len);

    return 0;
}

/**
 * ICM-42688 寄存器配置
 */
static int icm42688_register_config(void)
{
    uint8_t val = 0;

    // 读 who am i 寄存器
    int times = 0;
    while (val != IMU_ICM42688_ID) {
        icm42688_read_register(IMU_WHO_AM_I, &val);
        rt_kprintf("who-am-i: %x", val);
        if (++ times > 100)
            return -RT_ERROR;
    }
    // 设置 bank 0 区域寄存器
    icm42688_write_register(IMU_REG_BANK_SEL, 0);
    // 软复位
    icm42688_write_register(IMU_REG_BANK_SEL, 1);
    rt_thread_mdelay(100);
    //// fifo
    //// icm42688_write_register(IMU_REG_BANK_SEL, 0);
    //// icm42688_write_register(IMU_FIFO_CONFIG, 0x40);

    scale_factor_set(GFS_1000DPS, AFS_2G);
    // Gyro
    icm42688_write_register(IMU_REG_BANK_SEL, 0);
    val = GFS_1000DPS << 5;
    val |= ODR_100Hz;
    icm42688_write_register(IMU_GYRO_CONFIG0, val);
    // Accel
    icm42688_write_register(IMU_REG_BANK_SEL, 0);
    val = AFS_2G << 5;
    val |= ODR_100Hz;
    icm42688_write_register(IMU_ACCEL_CONFIG0, val);

    icm42688_write_register(IMU_REG_BANK_SEL, 0);
    icm42688_read_register(IMU_PWR_MGMT0, &val);
    val |= 3 << 2;    // gyro mode: 低噪声
    val |= 3;         // acc  mode: 低噪声
    icm42688_write_register(IMU_PWR_MGMT0, val);
    rt_thread_mdelay(1);

    return 0;
}

int imu_init(void)
{
    spi_init();

    // buffer alloc
    imu_tx = rt_malloc(IMU_READ_BUF_SIZE * sizeof(char));
    imu_rx = rt_malloc(IMU_READ_BUF_SIZE * sizeof(char));

    if (!imu_tx || !imu_rx)
        return -RT_ENOMEM;

    // icm42688 init
    if (icm42688_register_config())
        return -RT_ERROR;

    return 0;
}

/**
 * ICM-42688 陀螺仪数据 读
 */
void icm42688_get_gyro(struct imu_raw *gyro)
{
    uint8_t buf[6];

    icm42688_burst_read(IMU_GYRO_DATA_X1, buf, sizeof(buf));

    gyro->x = ((int16_t)(buf[0] << 8) | buf[1]) * gscale;
    gyro->y = ((int16_t)(buf[2] << 8) | buf[3]) * gscale;
    gyro->z = ((int16_t)(buf[4] << 8) | buf[5]) * gscale;
}

/**
 * ICM-42688 加速度计数据 读
 */
void icm42688_get_acc(struct imu_raw *acc)
{
    uint8_t buf[6];

    icm42688_burst_read(IMU_ACCEL_DATA_X1, buf, sizeof(buf));

    acc->x = ((int16_t)(buf[0] << 8) | buf[1]) * ascale;
    acc->y = ((int16_t)(buf[2] << 8) | buf[3]) * ascale;
    acc->z = ((int16_t)(buf[4] << 8) | buf[5]) * ascale;
}

void icm42688_get_gyro_acc(struct imu_raw *gyro, struct imu_raw *acc)
{
    uint8_t buf[12];

    icm42688_burst_read(IMU_ACCEL_DATA_X1, buf, 12);

    acc->x  = ((int16_t)(buf[0]  << 8) | buf[1] ) * ascale;
    acc->y  = ((int16_t)(buf[2]  << 8) | buf[3] ) * ascale;
    acc->z  = ((int16_t)(buf[4]  << 8) | buf[5] ) * ascale;
    gyro->x = ((int16_t)(buf[6]  << 8) | buf[7] ) * gscale;
    gyro->y = ((int16_t)(buf[8]  << 8) | buf[9] ) * gscale;
    gyro->z = ((int16_t)(buf[10] << 8) | buf[11]) * gscale;
}

/*
 * low-pass filter for accel
 */
static void low_pass_filter(struct imu_raw *acc, float alpha)
{
    static struct imu_raw filtered_acc;

    filtered_acc.x = alpha * acc->x + (1-alpha) * filtered_acc.x;
    filtered_acc.y = alpha * acc->y + (1-alpha) * filtered_acc.y;
    filtered_acc.z = alpha * acc->z + (1-alpha) * filtered_acc.z;

    acc->x = filtered_acc.x;
    acc->y = filtered_acc.y;
    acc->z = filtered_acc.z;
}

/**
 * 快速计算 1/sqrt(x)
 */
static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;

    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}

/* 
 * Mahony Filter
 */

#define IMU_Kp 0.5
#define IMU_Ki 0.006


void mahony_core(float q[4], struct imu_raw *gyro, struct imu_raw *acc, float half_dt)
{
    static float ex_int, ey_int, ez_int;
    float norm; // 归一化范数
    float vx, vy, vz; // 加速度矢量
    float ex, ey, ez; // 误差向量
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float ax = acc->x, ay = acc->y, az = acc->z;
    float gx = DEG2RAD(gyro->x), gy = DEG2RAD(gyro->y), gz = DEG2RAD(gyro->z);

    if (ax || ay || az) {
        // 加速度向量单位化
        norm = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // 通过四元数得到理论重力加速度在机体坐标系下的向量值
        vx = 2 * (q1 * q3 - q0 * q2);
        vy = 2 * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 误差: 测量值与估计值做外积
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        if (ex && ey && ez) {
            // 积分
            ex_int += IMU_Ki * ex * half_dt;
            ey_int += IMU_Ki * ey * half_dt;
            ez_int += IMU_Ki * ez * half_dt;
            // 误差修正
            gx += IMU_Kp * ex + ex_int;
            gy += IMU_Kp * ey + ey_int;
            gz += IMU_Kp * ez + ez_int;
        }

        // 角速度微分计算四元数
        q[0] -= ( q1 * gx + q2 * gy + q3 * gz) * half_dt;
        q[1] += ( q0 * gx - q3 * gy + q2 * gz) * half_dt;
        q[2] += ( q3 * gx + q0 * gy - q1 * gz) * half_dt;
        q[3] += (-q2 * gx + q1 * gy + q0 * gz) * half_dt; 

        // 单位化
        norm = inv_sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        q[0] *= norm;
        q[1] *= norm;
        q[2] *= norm;
        q[3] *= norm;
    }
}

void sensor_fusion(float q[4], struct imu_raw *gyro, struct imu_raw *acc)
{
    static uint32_t mf_tick;
    uint32_t current = HAL_GetTick();
    float half_dt;

    // filtering high-freq noise from accel
    low_pass_filter(acc, 0.2);

    // delta-t
    half_dt = qfp_fdiv(current - mf_tick, 2000);
    mf_tick = current;

    mahony_core(q, gyro, acc, half_dt);
}


/**
 * input format [w, x, y, z]
 * get yaw from quanternion
 * return radian [-π, π]
 */
float quat_to_yaw(const float q[4])
{
    const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    return qfp_fatan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3));
}