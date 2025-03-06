#ifndef SRC_FUNS
#define SRC_FUNS

#define USE_PRINTF_DMA

enum THREAD_PRIO {
    PRIO_EXTREME = 0,
    PRIO_VERY_HIGH = 5,
    PRIO_HIGH = 10,
    PRIO_MEDIUM = 15,
    PRIO_LOW = 20,
    PRIO_VERY_LOW = 25,
    PRIO_LOWEST = 31
};

enum STACK_SIZE {
    STACK_128B = 128,
    STACK_256B = 256,
    STACK_512B = 512,
    STACK_1KB = 1024,
    STACK_2KB = 2048,
    STACK_4KB = 4096,
    STACK_8KB = 8192
};


struct imu {
    float quanternion[4];
    float gyro_v[3];
    float accel_v[3];
};

struct odom {
    float position[2];
    float twist[2];
};

struct tx_pack {
    uint8_t header_a;
    uint8_t header_b;
    struct imu imu;
    struct odom odom;
    uint32_t crc;
} __attribute__((packed));

struct rx_pack {
    uint8_t header_a;
    uint8_t header_b;
    struct {
        float l_vel;
        float r_vel;
    } cmd;
    uint32_t crc;
} __attribute__((packed));

void led_onboard_init(void);
void lock_uart_init(void);
void ros_msg_uart_init(void);
void ros_message_transmit(const uint8_t *, uint16_t);
void ros_message_receive(uint8_t *, uint16_t);
void kdb_raw_data_transmit(uint8_t *, int);
void vofa_receive_it(void);

void crc_init(void);
uint32_t crc32_calculate(uint32_t *, int);

void voltage_adc_init(void);
float read_voltage(void);

/*
 * motor and encoder interfaces
 *
 * xxx_PERIOD: use the same frequency of reading speed in encoder 
 * and controlling motor
 */
struct pid {
    float error_prev;
    float error_int;
};

void gpio_ain_bin_init(void);
void pwm_generator_init(void);
void motor_drive_pwm_set(int, int);
void encoder_init(void);
void motor_speed_update();
float motor_speed_read(char);
void lock_speed_init(void);
int pid_corrector(float error, struct pid *x);
int pid_corrector_old(float, float, float *);

/*
 * imu 3-axis raw data
 * maybe read in registers, range -32768 - 32767 / s
 * or trans to dps when called imu_get_gyro
 */
struct imu_raw {
    int16_t x;
    int16_t y;
    int16_t z;
};

#define DEG2RAD(deg)    qfp_fmul(deg, 0.01745329)

int imu_init(void);
void icm42688_get_gyro(struct imu_raw *);
void icm42688_get_acc(struct imu_raw *);
void icm42688_get_gyro_acc(struct imu_raw *, struct imu_raw *);
void sensor_fusion(float *, struct imu_raw *, struct imu_raw *);
float quat_to_yaw(const float *);

#endif