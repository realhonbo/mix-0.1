#include <stdio.h>
#include <rtthread.h>
#include <qfplib-m3.h>
#include "gpio.h"
#include "include/multi.h"

// distance between wheels
#define ROBOT_WHEEL_D   0.18

static float quat[4] = { 1, 0, 0, 0 };
static struct tx_pack tx_ros;
struct rx_pack rx_ros;


//TODO: healthy working and tb6612 input voltage indicator
void basic_monitor()
{
    led_onboard_init();
    voltage_adc_init();

    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        printf("voltage %fV\r\n", read_voltage());

        rt_thread_mdelay(500);
    }
}

//TODO: correct by pid, output pwm, and produce odometry
void dual_motor_controller()
{
    int l_pwm = 0, r_pwm = 0;
    float l_actual, r_actual;
    float l_error[3] = {0};
    float r_error[3] = {0};
    static int last_tick;
    float dt;
    float l_vel, r_vel;
    float yaw;

    gpio_ain_bin_init();
    pwm_generator_init();
    encoder_init();
    ros_message_receive_it((uint8_t *)&rx_ros, sizeof(rx_ros));

    while (1) {
/* motor control */
        printf("%2.3f %2.3f\r\n", rx_ros.cmd.l_vel, rx_ros.cmd.r_vel);
        //kdb_raw_data_transmit((uint8_t *)&rx_ros, 14);

        l_actual = motor_speed_read('a');
        r_actual = motor_speed_read('b');

        l_pwm += pid_corrector_old(rx_ros.cmd.l_vel, l_actual, l_error);
        r_pwm += pid_corrector_old(rx_ros.cmd.r_vel, r_actual, r_error);

        motor_drive_pwm_set(l_pwm, r_pwm);

/* odometry producer */
        motor_speed_update();

        l_vel = motor_speed_read('a');
        r_vel = motor_speed_read('b');
        yaw = quat_to_yaw(quat);
        dt = qfp_fdiv(HAL_GetTick() - last_tick, 1000);

        // fill odometry
        tx_ros.odom.twist[0] = qfp_fdiv(l_vel + r_vel, 2);
        tx_ros.odom.twist[1] = qfp_fdiv(r_vel - l_vel, ROBOT_WHEEL_D);
        tx_ros.odom.position[0] += tx_ros.odom.twist[0] * qfp_fcos(yaw) * dt;
        tx_ros.odom.position[1] += tx_ros.odom.twist[0] * qfp_fsin(yaw) * dt;

        last_tick = HAL_GetTick();

        rt_thread_mdelay(50);
    }
}


//TODO: icm42688 decode raw data to quaternion
void imu_data_producer()
{
    struct imu_raw gyro, accel;

    imu_init();

    while (1) {
        icm42688_get_gyro_acc(&gyro, &accel);
        sensor_fusion(quat, &gyro, &accel);

        // fill imu-data
        tx_ros.imu.quanternion[0] = quat[0];
        tx_ros.imu.quanternion[1] = quat[1];
        tx_ros.imu.quanternion[2] = quat[2];
        tx_ros.imu.quanternion[3] = quat[3];
        tx_ros.imu.gyro_v[0] = DEG2RAD(gyro.x);
        tx_ros.imu.gyro_v[1] = DEG2RAD(gyro.y);
        tx_ros.imu.gyro_v[2] = DEG2RAD(gyro.z);
        tx_ros.imu.accel_v[0] = DEG2RAD(accel.x);
        tx_ros.imu.accel_v[1] = DEG2RAD(accel.y);
        tx_ros.imu.accel_v[2] = DEG2RAD(accel.z);

        rt_thread_mdelay(10);
    }
}


//TODO: transmit message to ros
int main(void)
{
    rt_thread_t t_basic, t_motor, t_imu;

    // resource lock
    lock_speed_init();
    lock_uart_init();

    // thread init
    t_basic = rt_thread_create("led", basic_monitor,
        NULL, STACK_1KB, PRIO_LOWEST, 10);
    t_motor = rt_thread_create("motor", dual_motor_controller,
        NULL, STACK_2KB, PRIO_VERY_HIGH, 50);
    t_imu = rt_thread_create("imu", imu_data_producer,
        NULL, STACK_2KB, PRIO_EXTREME, 50);

    if ( !t_basic || !t_motor || !t_imu) {
        rt_kprintf("Failed in thread create: 0x%x  0x%x  0x%x", t_basic, t_motor, t_imu);
        return -RT_ERROR;
    }

    rt_thread_startup(t_basic);
    rt_thread_startup(t_motor);
    rt_thread_startup(t_imu);

    ros_msg_uart_init();
    crc_init();

    while (1) {
        // tx: fill header and crc
        tx_ros.header_a = 0xff;
        tx_ros.header_b = 0xff;
        tx_ros.crc = crc32_calculate((uint32_t *)&tx_ros.imu, 
            (sizeof(struct imu) + sizeof(struct odom)) / 4
        );
        ros_message_transmit((uint8_t *)&tx_ros, sizeof(struct tx_pack));

        rt_thread_mdelay(10);
    }
}
