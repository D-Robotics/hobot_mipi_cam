#include "inv_icm42600.h"

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>           // open()
#include <unistd.h>          // close(), read(), write()
#include <sys/ioctl.h>       // ioctl()
#include <linux/i2c-dev.h>   // I2C 设备接口
#include <linux/i2c.h>       // I2C 消息结构体
#include <time.h>            // 时间戳
#include <linux/gpio.h>
#include <string.h>

#define INV_ICM42600_SENSOR_STATE_INIT { \
    /* DEVICE_CONFIG */ \
    /*.bank_id = 0,*/ \
    .spi_mode = 0, \
    /* DRIVE_CONFIG */ \
    .i2c_slew_rate = INV_ICM42600_SLEW_RATE_20NS_60NS, \
    .spi_slew_rate = INV_ICM42600_SLEW_RATE_LESS_2NS, \
    /* FIFO_CONFIG */ \
    .fifo_mode = INV_ICM42600_STOP_ON_FULL_MODE, \
    /* INTF_CONFIG0 */ \
    .fifo_hold_last_data_en = false, \
    .fifo_count_rec = false, \
    .fifo_count_be = true, \
    .sensor_data_be = true, \
    .ui = INV_ICM42600_DISABLE_SPI, \
    /* PWR_MGMT0 */ \
    .temp_sensor_pwr_en = false, \
    .gyro_pwr_mode = INV_ICM_42600_GYRO_OFF, \
    .accel_pwr_mode = INV_ICM_42600_ACCEL_OFF, \
    /* GYRO_CONFIG0 */ \
    .gyro_fs = INV_ICM42600_GYRO_FS_2000DPS, \
    .gyro_odr = INV_ICM42600_GYRO_ODR_8KHZ, \
    /* ACCEL_CONFIG0  */ \
    .accel_fs = INV_ICM42600_ACCEL_FS_16G, \
    .accel_odr = INV_ICM42600_ACCEL_ODR_8KHZ_LN, \
    /* GYRO_CONFIG1 */ \
    .temp_filt_bw = INV_ICM42600_DLPF_4000HZ_0_125MS, \
    .gyro_ui_filt_ord = 1, \
    .gyro_dec2_m2_ord = 2, \
    /* GYRO_ACCEL_CONFIG0 */ \
    .accel_filt_bw = BW_ODR_DIR_40, \
    .accel_LP_filt_avg = AVG_1_FILTER, \
    .gyro_filt_bw = BW_ODR_DIR_40, \
    /* TMST_CONFIG */ \
    .tmst_en = true, \
    .tmst_fsync_en = true, \
    .tmst_delta_en = true, \
    .tmst_res_en = false, \
    .tmst_to_regs_en = true, \
    /* FIFO_CONFIG1 */ \
    .fifo_accel_en = true, \
    .fifo_gyro_en = true, \
    .fifo_temp_en = true, \
    .fifo_tmst_fsync = false, \
    .fifo_hires_en = false, \
    .fifo_wm_gt_th = false, \
    .fifo_resume_partial_rd = false, \
    /* FIFO_CONFIG2 FIFO_CONFIG3 */ \
    .fifo_watermark = 0x0000, \
    /* FSYNC_CONFIG */ \
    .fsync_ui_sel = 0x1, \
    .fsync_ui_flag_clear = true, \
    .fsync_polarity = false, \
    /* INTF_CONFIG5(FSYNC_PIN) */ \
    .pin9_func = PIN9_FUNCTION_FSYNC, \
    /* INTF_CONFIG1 涉及时钟设置，跟timestamp初始化放在一起 */ \
    .accel_lp_clk_sel = true, \
    .rtc_mode = false, \
    .clksel = 0, \
}

static const float gyro_fs_value[INV_ICM42600_GYRO_FS_NUM] = { /* 单位：dps */
    /*0*/2000.0f, 
    /*1*/1000.0f, 
    /*2*/500.0f, 
    /*3*/250.0f, 
    /*4*/125.0f, 
    /*5*/62.5f, 
    /*6*/31.25f, 
    /*7*/15.625f}; 

static const float gyro_odr_value[INV_ICM42600_GYRO_ODR_NUM] = {
    /*0*/0.0f,
    /*1*/32000.0f,
    /*2*/16000.0f,
    /*3*/8000.0f,
    /*4*/4000.0f,
    /*5*/2000.0f,
    /*6*/1000.0f,
    /*7*/200.0f,
    /*8*/100.0f,
    /*9*/50.0f,
    /*10*/25.0f,
    /*11*/12.5f,
    /*12*/0.0,
    /*13*/0.0,
    /*14*/0.0,
    /*15*/500.0f};

static const float accel_fs_value[INV_ICM42600_ACCEL_FS_NUM] = { /* 单位：dps */
    /*0*/16.0f, 
    /*1*/8.0f, 
    /*2*/4.0f, 
    /*3*/2.0f}; 

static const float accel_odr_value[INV_ICM42600_ACCEL_ODR_NUM] = {
    /*0*/0.0,
    /*1*/32000.0f,
    /*2*/16000.0f,
    /*3*/8000.0f,
    /*4*/4000.0f,
    /*5*/2000.0f,
    /*6*/1000.0f,
    /*7*/200.0f,
    /*8*/100.0f,
    /*9*/50.0f,
    /*10*/25.0f,
    /*11*/12.5f,
    /*12*/6.25f,
    /*13*/3.125f,
    /*14*/1.5625f,
    /*15*/500.0f};

static int inv_icm_42600_i2c_fd;
static int inv_icm_42600_i2c_addr;
static struct inv_icm42600_sensor_state inv_icm42600_st = INV_ICM42600_SENSOR_STATE_INIT;

static int fsync_gpio_chip_fd;      // GPIO芯片文件描述符
static int fsync_gpio_line_fd;      // GPIO线路文件描述符
static int fsync_gpio_line_offset;  // GPIO线路偏移量

static int i2c_read_register(
    int i2c_fd, 
    uint8_t device_addr, 
    uint8_t reg_addr, 
    uint8_t *buffer, 
    int len)
{
    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data packet;
    
    // 第一条消息：写入寄存器地址
    messages[0].addr = device_addr;   // 设备地址
    messages[0].flags = 0;            // 写操作
    messages[0].len = 1;              // 寄存器地址长度
    messages[0].buf = &reg_addr;      // 寄存器地址
    
    // 第二条消息：读取数据
    messages[1].addr = device_addr;   // 设备地址
    messages[1].flags = I2C_M_RD;     // 读操作
    messages[1].len = len;            // 要读取的字节数
    messages[1].buf = buffer;         // 数据缓冲区
    
    // 设置 I2C_RDWR 数据结构
    packet.msgs = messages;
    packet.nmsgs = 2;
    
    // 执行组合的读写操作
    if (ioctl(i2c_fd, I2C_RDWR, &packet) < 0) {
        // perror("I2C_RDWR failed");
        return -1;
    }
    
    return 0;
}

static int i2c_write_register(
    int i2c_fd, 
    uint8_t device_addr,                  
    uint8_t reg_addr, 
    uint8_t *data, 
    int len) 
{
    struct i2c_msg message;
    struct i2c_rdwr_ioctl_data packet;
    uint8_t buffer[len + 1];
    
    // 构造数据缓冲区：寄存器地址 + 数据
    buffer[0] = reg_addr;
    for (int i = 0; i < len; i++) {
        buffer[i + 1] = data[i];
    }
    
    // 设置消息
    message.addr = device_addr;
    message.flags = 0;  // 写操作
    message.len = len + 1;
    message.buf = buffer;
    
    // 设置 I2C_RDWR 数据结构
    packet.msgs = &message;
    packet.nmsgs = 1;
    
    // 执行写操作
    if (ioctl(i2c_fd, I2C_RDWR, &packet) < 0) {
        // perror("I2C_RDWR write failed");
        return -1;
    }
    
    return 0;
}

static inline int i2c_read_bank_register(
    int i2c_fd, 
    uint8_t device_addr, 
    uint16_t bank_reg_addr, 
    uint8_t *buffer, 
    int len)
{   
    uint8_t bank_id = bank_reg_addr >> 8;
    uint8_t reg_addr = bank_reg_addr & 0xFF;
    /* 需要进行bank切换 */
    if(bank_id != 0){
        if(i2c_write_register(i2c_fd, device_addr, INV_ICM42600_REG_BANK_SEL, &bank_id, 1) != 0)return -1;
    }
    /* 常规读取 */
    if(i2c_read_register(i2c_fd, device_addr, reg_addr, buffer, len) != 0)return -1;
    /* 切换回bank0 */
    if(bank_id != 0){
        bank_id = 0;
        if(i2c_write_register(i2c_fd, device_addr, INV_ICM42600_REG_BANK_SEL, &bank_id, 1) != 0)return -1;
    }

    return 0;
}

static inline int i2c_write_bank_register(
    int i2c_fd, 
    uint8_t device_addr, 
    uint16_t bank_reg_addr, 
    uint8_t *data, 
    int len)
{   
    uint8_t bank_id = bank_reg_addr >> 8;
    uint8_t reg_addr = bank_reg_addr & 0xFF;
    /* 需要进行bank切换 */
    if(bank_id != 0){
        if(i2c_write_register(i2c_fd, device_addr, INV_ICM42600_REG_BANK_SEL, &bank_id, 1) != 0)return -1;
    }
    /* 常规写入 */
    if(i2c_write_register(i2c_fd, device_addr, reg_addr, data, len) != 0)return -1;
    /* 切换回bank0 */
    if(bank_id != 0){
        bank_id = 0;
        if(i2c_write_register(i2c_fd, device_addr, INV_ICM42600_REG_BANK_SEL, &bank_id, 1) != 0)return -1;
    }

    return 0;
}

int inv_icm42600_i2c_fd_auto_init(){
    int fd;
    int ret;
    char device_name[16];

    uint8_t who_am_i_value;

    for(int i=0; i<8; i++){
        /* 正确构建设备文件名 */
        snprintf(device_name, sizeof(device_name), "/dev/i2c-%d", i);
        /* 打开设备文件 */
        fd = open(device_name, O_RDWR);
        if(fd<0){
            printf("[inv_icm_42600_i2c_fd_auto_init]Failed to open device file: /dev/i2c-%d.\n", i);
            continue;
        }
        else printf("[inv_icm_42600_i2c_fd_auto_init]Successfully opened device file: /dev/i2c-%d.\n", i);

        /* 地址扫描，使用icm42688对应的WHO_AM_I寄存器校对 */
        for(uint8_t addr=0; addr<0x80; addr++){

            ret = i2c_read_bank_register(fd, addr, INV_ICM42600_REG_WHO_AM_I, &who_am_i_value, 1);
            if(ret == 0 && who_am_i_value == INV_ICM42600_WHO_AM_I_VALUE){ /* 扫描到正确地址和设备了 */
                printf("[inv_icm_42600_i2c_fd_auto_init]Device successfully scanned, in i2c-%d bus, device addr: %#x\n", i, addr);
                inv_icm_42600_i2c_fd = fd;
                inv_icm_42600_i2c_addr = addr;

                return 0;
            }
        }
        printf("[inv_icm_42600_i2c_fd_auto_init]icm_42600 was not found on i2c-%d bus.\n", i);
        close(fd);
    }

    return -1;
}

static int inv_icm42600_fsync_gpio_sel(
    struct inv_icm42600_sensor_state *st)
{
    char chip_path[32];
    snprintf(chip_path, sizeof(chip_path), "/dev/gpiochip%d", FSYNC_GPIO_CHIP_NUM);
    
    // 打开GPIO芯片
    fsync_gpio_chip_fd = open(chip_path, O_RDWR);
    if (fsync_gpio_chip_fd < 0)return -1;

    // 准备请求结构
    struct gpio_v2_line_request req;
    memset(&req, 0, sizeof(req));
    
    req.offsets[0] = FSYNC_GPIO_CHIP_LINE_OFFSET;
    req.num_lines = 1;
    strncpy(req.consumer, "User@inv-icm42600 fsync gpio port", sizeof(req.consumer) - 1);
    
    // 3. 配置为输出模式
    struct gpio_v2_line_config config;
    memset(&config, 0, sizeof(config));
    
    // 设置基本标志：输出模式
    config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    
    // 设置初始输出值（通过属性）
    config.num_attrs = 1;
    config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
    config.attrs[0].attr.values = st->fsync_polarity?1UL:0UL;
    config.attrs[0].mask = 1UL;
    
    req.config = config;
    
    // 请求GPIO线路控制权
    if (ioctl(fsync_gpio_chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0 || req.fd < 0) {
        close(fsync_gpio_chip_fd);
        return -2;
    }
    
    fsync_gpio_line_fd = req.fd;
    fsync_gpio_line_offset = FSYNC_GPIO_CHIP_LINE_OFFSET;
    
    // printf("GPIO初始化成功: 芯片%d, 线路%d, 初始值=0\n", 
    //        FSYNC_GPIO_CHIP_NUM, fsync_gpio_line_offset);
    
    return 0;
}

static int gpio_output_set(
    struct inv_icm42600_sensor_state *st,
    bool fsync_trigger) {

    struct gpio_v2_line_values vals;
    memset(&vals, 0, sizeof(vals));
    
    vals.mask = 1UL;  // 操作第0位（第一条线）
    vals.bits = fsync_trigger ? !st->fsync_polarity : st->fsync_polarity;  // 设置值
    
    if(ioctl(fsync_gpio_line_fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals) < 0)return -1;
    
    return 0;
}

static inline int inv_icm42600_spi_mode_sel(
    struct inv_icm42600_sensor_state *st)
{
    uint8_t data = st->spi_mode!=0 ? DEVICE_CONFIG_SPI_MODE_MASK : 0;
    
    return i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_DEVICE_CONFIG, /* DEVICE_CONFIG */
        &data, 
        1);
}

static inline int inv_icm42600_slew_rate_sel(
    struct inv_icm42600_sensor_state *st)
{
    uint8_t data = 
        DRIVE_CONFIG_I2C_SLEW_RATE(st->i2c_slew_rate) |
        DRIVE_CONFIG_SPI_SLEW_RATE(st->spi_slew_rate);

    return i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_DRIVE_CONFIG, /* DRIVE_CONFIG */
        &data, 
        1);
}

static inline int inv_icm42600_fifo_sel(
    struct inv_icm42600_sensor_state *st)
{
    int ret;
    uint8_t data;
    
    data = FIFO_CONFIG_FIFO_MODE(st->fifo_mode);
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_FIFO_CONFIG, /* FIFO_CONFIG */
        &data, 
        1);
    if(ret<0)return -1;

    data = (st->fifo_hold_last_data_en ? FIFO_HOLD_LAST_DATA_EN_MASK : 0) |
        (st->fifo_count_rec ? FIFO_COUNT_REC_MASK : 0) |
        (st->fifo_count_be ? FIFO_COUNT_ENDIAN_MASK : 0) |
        (st->sensor_data_be ? SENSOR_DATA_ENDIAN_MASK : 0) |
        INTF_CONFIG0_UI_SIFS_CFG(st->ui);
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_INTF_CONFIG0, /* INTF_CONFIG0 */
        &data, 
        1);
    if(ret<0)return -2;

    data = (st-> fifo_resume_partial_rd ? FIFO_RESUME_PARTIAL_RD_MASK : 0) |
        (st->fifo_wm_gt_th ? FIFO_WM_GT_TH_MASK : 0) |
        (st->fifo_hires_en ? FIFO_HIRES_EN_MASK : 0) |
        (st->fifo_tmst_fsync ? FIFO_TMST_FSYNC_EN_MASK : 0) |
        (st->fifo_temp_en ? FIFO_TEMP_EN_MASK : 0) |
        (st->fifo_gyro_en ? FIFO_GYRO_EN_MASK : 0) |
        (st->fifo_accel_en ? FIFO_ACCEL_EN_MASK : 0);
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_FIFO_CONFIG1, 
        &data, 
        1);
    if(ret<0)return -3;

    data = st->fifo_watermark & 0xFF;
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_FIFO_CONFIG2, 
        &data, 
        1);
    if(ret<0)return -4;

    data = st->fifo_watermark >> 8;
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_FIFO_CONFIG3, 
        &data, 
        1);
    if(ret<0)return -5;

    return ret;
}

static inline int inv_icm42600_gyro_and_accel_sel(
    struct inv_icm42600_sensor_state *st)
{
    int ret;
    uint8_t data;

    data = GYRO_CONFIG0_FS(st->gyro_fs) | GYRO_CONFIG0_ODR(st->gyro_odr);
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_GYRO_CONFIG0, 
        &data, 
        1);
    if(ret<0)return -1;

    data = ACCEL_CONFIG0_FS(st->accel_fs) | ACCEL_CONFIG0_ODR(st->accel_odr);
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_ACCEL_CONFIG0, 
        &data, 
        1);
    if(ret<0)return -2;

    data = GYRO_CONFIG1_TEMP_FILT_BW(st->temp_filt_bw) |
        GYRO_CONFIG1_GYRO_UI_FILT_ORD(st->gyro_ui_filt_ord | 0x03) |
        GYRO_CONFIG1_GYRO_DEC2_M2_ORD(st->gyro_dec2_m2_ord | 0x03);
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_GYRO_CONFIG1, 
        &data, 
        1);
    if(ret<0)return -3;

    return ret;
}

static inline int inv_icm42600_timestamp_fsync_sel(
    struct inv_icm42600_sensor_state *st)
{
    int ret;
    uint8_t data;

    data = (st->tmst_to_regs_en ? TMST_TO_REGS_EN_MASK : 0) |
        (st->tmst_res_en ? TMST_RES_MASK : 0) |
        (st->tmst_delta_en ? TMST_DELTA_EN_MASK : 0) |
        (st->tmst_fsync_en ? TMST_FSYNC_EN_MASK : 0) |
        (st->tmst_en ? TMST_EN_MASK : 0);
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_TMST_CONFIG, 
        &data, 
        1);
    if(ret<0)return -1;

    data = FSYNC_CONFIG_UI_SEL(st->fsync_ui_sel) |
        (st->fsync_ui_flag_clear ? FSYNC_UI_FLAG_CLEAR_SEL_MASK : 0) | 
        (st->fsync_polarity ? FSYNC_POLARITY_MASK : 0);
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_FSYNC_CONFIG, 
        &data, 
        1);
    if(ret<0)return -1;

    data = INTF_CONFIG5_PIN9_FUNCTION(st->pin9_func);
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_INTF_CONFIG5, 
        &data, 
        1);
    if(ret<0)return -1;

    data = 0x90 |
        (st->accel_lp_clk_sel ? ACCEL_LP_CLK_SEL_MASK : 0) |
        (st->rtc_mode ? RTC_MODE_MASK : 0) |
        INTF_CONFIG1_CLKSEL(st->clksel);
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_INTF_CONFIG1, 
        &data, 
        1);
    if(ret<0)return -1;

    return ret;
}

int inv_icm42600_all_sel()
{
    int ret;

    ret = inv_icm42600_fsync_gpio_sel(&inv_icm42600_st);
    if(ret<0){
        printf("[inv_icm42600_all_sel]Failed to set fsync_gpio, inv_icm42600_fsync_gpio_sel return %d.\n", ret);
        return -1;
    }
    printf("[inv_icm42600_all_sel]Successfully to set fsync_gpio, inv_icm42600_fsync_gpio_sel return %d.\n", ret);

    ret = inv_icm42600_spi_mode_sel(&inv_icm42600_st);
    if(ret<0){
        printf("[inv_icm42600_all_sel]Failed to set spi_mode, inv_icm42600_spi_mode_sel return %d.\n", ret);
        return -2;
    }
    printf("[inv_icm42600_all_sel]Successfully to set spi_mode, inv_icm42600_spi_mode_sel return %d.\n", ret);

    ret = inv_icm42600_slew_rate_sel(&inv_icm42600_st);
    if(ret<0){
        printf("[inv_icm42600_all_sel]Failed to set slew_rate, inv_icm42600_slew_rate_sel return %d.\n", ret);
        return -3;
    }
    printf("[inv_icm42600_all_sel]Successfully to set slew_rate, inv_icm42600_slew_rate_sel return %d.\n", ret);

    ret = inv_icm42600_fifo_sel(&inv_icm42600_st);
    if(ret<0){
        printf("[inv_icm42600_all_sel]Failed to set fifo, inv_icm42600_fifo_sel return %d.\n", ret);
        return -4;
    }
    printf("[inv_icm42600_all_sel]Successfully to set fifo, inv_icm42600_fifo_sel return %d.\n", ret);

    ret = inv_icm42600_gyro_and_accel_sel(&inv_icm42600_st);
    if(ret<0){
        printf("[inv_icm42600_all_sel]Failed to set gyro_and_accel, inv_icm42600_gyro_and_accel_sel return %d.\n", ret);
        return -5;
    }
    printf("[inv_icm42600_all_sel]Successfully to set gyro_and_accel, inv_icm42600_gyro_and_accel_sel return %d.\n", ret);

    ret = inv_icm42600_timestamp_fsync_sel(&inv_icm42600_st);
    if(ret<0){
        printf("[inv_icm42600_all_sel]Failed to set timestamp_fsync, inv_icm42600_timestamp_fsync_sel return %d.\n", ret);
        return -6;
    }
    printf("[inv_icm42600_all_sel]Successfully to set timestamp_fsync, inv_icm42600_timestamp_fsync_sel return %d.\n", ret);

    return ret;
}

int inv_icm42600_pwr_mgmt(
    enum inv_icm42600_pwr_mgmt0_gyro_mode gyro_pwr,
    enum inv_icm42600_pwr_mgmt0_accel_mode accel_pwr,
    bool temp_pwr,
    bool RC_clock_pwr)
{
    uint8_t data, buff;
    int ret;

    /* 将状态更新到结构体中 */
    inv_icm42600_st.gyro_pwr_mode = gyro_pwr;
    inv_icm42600_st.accel_pwr_mode = accel_pwr;
    inv_icm42600_st.temp_sensor_pwr_en = temp_pwr;

    usleep(1000*(INV_ICM_42600_GYRO_OPENED_DELAY_MS > INV_ICM_42600_ACCEL_OPENED_DELAY_MS ? INV_ICM_42600_GYRO_OPENED_DELAY_MS : INV_ICM_42600_ACCEL_OPENED_DELAY_MS));
    
    data = PWR_MGMT0_GYRO_MODE(gyro_pwr) | 
        PWR_MGMT0_ACCEL_MODE(accel_pwr) | 
        (temp_pwr ? 0 : TEMP_DIS_MASK) |
        (RC_clock_pwr ? IDLE_MASK : 0);

    /* 操作寄存器，控制实际电源状态 */
    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_PWR_MGMT0, 
        &data, 
        1);
    if(ret < 0)return -1;
    
    usleep(1000*(INV_ICM_42600_ACCEL_CLOSED_DELAY_MS > INV_ICM_42600_GYRO_CLOSED_DELAY_MS ? INV_ICM_42600_ACCEL_CLOSED_DELAY_MS : INV_ICM_42600_GYRO_CLOSED_DELAY_MS));

    /* 查看写入是否有效 */
    ret = i2c_read_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_PWR_MGMT0, 
        &buff, 
        1);
    if(ret < 0)return -2;

    if(buff != data)return -3;

    return 0;
}

int inv_icm42600_filt_setting()
{
    uint8_t data = 0;
    uint8_t buff;
    int ret;

    data |= GYRO_UI_FILT_BW(inv_icm42600_st.gyro_filt_bw);
    if(inv_icm42600_st.accel_pwr_mode == INV_ICM_42600_ACCEL_LP)
        data |= ACCEL_UI_FILT_BW(inv_icm42600_st.accel_LP_filt_avg);
    else
        data |= ACCEL_UI_FILT_BW(inv_icm42600_st.accel_filt_bw);

    ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_GYRO_ACCEL_CONFIG0, 
        &data, 
        1);
    if(ret < 0)return -1;

    /* 查看写入是否有效 */
    ret = i2c_read_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_GYRO_ACCEL_CONFIG0, 
        &buff, 
        1);
    if(ret < 0)return -2;

    if(buff != data)return -3;

    return 0;
}

int inv_icm42600_soft_reset()
{
    uint8_t reset_data = DEVICE_CONFIG_SOFT_RESET_CONFIG_MASK;

    int ret = i2c_write_bank_register(
        inv_icm_42600_i2c_fd,
        inv_icm_42600_i2c_addr,
        INV_ICM42600_REG_DEVICE_CONFIG, 
        &reset_data, 
        1);

    usleep(10000);

    return ret;
}

static inline uint64_t get_timestamp_us_realtime()
{
    struct timespec ts;
    // CLOCK_REALTIME: 系统实际时间，可被NTP或用户调整
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

int inv_icm42600_origin_data_read(
    struct inv_icm42600_data_packege *data_pkt,
    bool try_fsync)
{
    int ret;
    uint8_t buffer[16];
    uint64_t fsync_tmst;
    uint32_t fsync_timeout=0xFFFF;
    uint8_t fsync_flag;

    data_pkt->fsync_success = false;

    if(!try_fsync){ // 普通读取
        ret = i2c_read_bank_register(
            inv_icm_42600_i2c_fd,
            inv_icm_42600_i2c_addr,
            INV_ICM42600_REG_TEMP_DATA1, 
            buffer, 
            16);
        if(ret < 0)return -1;

        data_pkt->temp_origin = buffer[0]<<8 | buffer[1];

        data_pkt->accel_data_origin.x = buffer[2]<<8 | buffer[3];
        data_pkt->accel_data_origin.y = buffer[4]<<8 | buffer[5];
        data_pkt->accel_data_origin.z = buffer[6]<<8 | buffer[7];

        data_pkt->gyro_data_origin.x = buffer[8]<<8 | buffer[9];
        data_pkt->gyro_data_origin.y = buffer[10]<<8 | buffer[11]; 

        data_pkt->src_reg = true;
        data_pkt->src_fifo = false;

        return 0;
    }

    /* 以下为fsync尝试 */

    /* fsync触发 */
    gpio_output_set(&inv_icm42600_st, true);
    /* 打入微秒级实时时间戳 */ 
    fsync_tmst = get_timestamp_us_realtime();
    data_pkt->gyro_data_origin.fsync_timestamp = fsync_tmst;
    data_pkt->accel_data_origin.fsync_timestamp = fsync_tmst;

    /* 根据时间戳单位获取超时时间 */ 
    if(inv_icm42600_st.tmst_res_en)fsync_timeout=fsync_timeout<<4;
    /* 在超时时间内重复读取 */
    while(get_timestamp_us_realtime() - fsync_tmst < fsync_timeout){
        /* 读取完整数据 */
        ret = i2c_read_bank_register(
            inv_icm_42600_i2c_fd,
            inv_icm_42600_i2c_addr,
            INV_ICM42600_REG_TEMP_DATA1, 
            buffer, 
            16);
        if(ret < 0)return -1;

        switch(inv_icm42600_st.fsync_ui_sel){
            case 1:
                // printf("testpoint %d\n",buffer[1] & 1);
                fsync_flag = buffer[1] & 1;
                break;
            case 5:
                fsync_flag = buffer[3] & 1;
                break;
            case 6:
                fsync_flag = buffer[5] & 1;
                break;
            case 7:
                fsync_flag = buffer[7] & 1;
                break;
            case 2:
                fsync_flag = buffer[9] & 1;
                break;
            case 3:
                fsync_flag = buffer[11] & 1;
                break;
            case 4:
                fsync_flag = buffer[13] & 1;
                break;
            default:
                return -2;
        }

        if(fsync_flag==1){
            data_pkt->fsync_success = true;
            break;
        }
    }

    /* fsync触发复原 */
    gpio_output_set(&inv_icm42600_st, false);

    data_pkt->temp_origin = buffer[0]<<8 | buffer[1];

    data_pkt->accel_data_origin.x = buffer[2]<<8 | buffer[3];
    data_pkt->accel_data_origin.y = buffer[4]<<8 | buffer[5];
    data_pkt->accel_data_origin.z = buffer[6]<<8 | buffer[7];

    data_pkt->gyro_data_origin.x = buffer[8]<<8 | buffer[9];
    data_pkt->gyro_data_origin.y = buffer[10]<<8 | buffer[11];
    data_pkt->gyro_data_origin.z = buffer[12]<<8 | buffer[13];

    data_pkt->accel_data_origin.deltatime = buffer[14]<<8 | buffer[15];
    data_pkt->gyro_data_origin.deltatime = buffer[14]<<8 | buffer[15];

    data_pkt->src_reg = true;
    data_pkt->src_fifo = false;

    return 0;
}

int inv_icm42600_data_process(
    struct inv_icm42600_data_packege *data_pkt)
{
    /* 温度处理 */
    if(data_pkt->src_reg)data_pkt->temp_processed = (data_pkt->temp_origin / 132.48f) + 25.0f;
    if(data_pkt->src_fifo)data_pkt->temp_processed = (data_pkt->temp_origin / 2.07) + 25;

    /* 6轴无效数据处理 */
    data_pkt->gyro_data_processed.processed = true;
    data_pkt->accel_data_processed.processed = true;
    if(!inv_icm42600_st.fifo_hold_last_data_en){
        if(data_pkt->gyro_data_origin.x == -32768 || data_pkt->gyro_data_origin.y == -32768 || data_pkt->gyro_data_origin.z == -32768)
            data_pkt->gyro_data_processed.processed = false;
    }
    if(!inv_icm42600_st.fifo_hold_last_data_en){
        if(data_pkt->accel_data_origin.x == -32768 || data_pkt->accel_data_origin.y == -32768 || data_pkt->accel_data_origin.z == -32768)
            data_pkt->accel_data_processed.processed = false;
    }

    /* gyro数据处理 */
    if(data_pkt->gyro_data_processed.processed){
        data_pkt->gyro_data_processed.x = gyro_fs_value[inv_icm42600_st.gyro_fs] * data_pkt->gyro_data_origin.x / 32768;
        data_pkt->gyro_data_processed.y = gyro_fs_value[inv_icm42600_st.gyro_fs] * data_pkt->gyro_data_origin.y / 32768;
        data_pkt->gyro_data_processed.z = gyro_fs_value[inv_icm42600_st.gyro_fs] * data_pkt->gyro_data_origin.z / 32768;
        if(data_pkt->fsync_success && data_pkt->gyro_data_origin.fsync_timestamp!=0)
            data_pkt->gyro_data_processed.timestamp = data_pkt->gyro_data_origin.fsync_timestamp + data_pkt->gyro_data_origin.deltatime;
    }
    
    /* accel数据处理 */
    if(data_pkt->accel_data_processed.processed){
        data_pkt->accel_data_processed.x = accel_fs_value[inv_icm42600_st.accel_fs] * data_pkt->accel_data_origin.x / 32768;
        data_pkt->accel_data_processed.y = accel_fs_value[inv_icm42600_st.accel_fs] * data_pkt->accel_data_origin.y / 32768;
        data_pkt->accel_data_processed.z = accel_fs_value[inv_icm42600_st.accel_fs] * data_pkt->accel_data_origin.z / 32768;
        if(data_pkt->fsync_success && data_pkt->accel_data_origin.fsync_timestamp!=0)
            data_pkt->accel_data_processed.timestamp = data_pkt->accel_data_origin.fsync_timestamp + data_pkt->accel_data_origin.deltatime;
    }

    return 0;
}

// int main(void){
//     int ret;

//     ret = inv_icm42600_i2c_fd_auto_init();
//     printf("[inv_icm42600_i2c_fd_auto_init]run return: %d.\n",ret);

//     ret = inv_icm42600_soft_reset();
//     printf("[inv_icm42600_soft_reset]run return: %d.\n",ret);

//     ret = inv_icm42600_all_sel();
//     printf("[inv_icm42600_all_sel]run return: %d.\n",ret);

//     ret = inv_icm42600_pwr_mgmt(
//         INV_ICM_42600_GYRO_LN,
//         INV_ICM_42600_ACCEL_LN,
//         true,
//         true);
//     printf("[inv_icm42600_pwr_mgmt]run return: %d.\n",ret);

//     ret = inv_icm42600_filt_setting();
//     printf("[inv_icm42600_filt_setting]run return: %d.\n",ret);

//     // usleep(1000000);

//     uint8_t buff[2];
//     uint8_t fifo_data[2080];
//     uint16_t fifo_count;

//     while(true){
//         ret = i2c_read_bank_register(
//             inv_icm_42600_i2c_fd,
//             inv_icm_42600_i2c_addr,
//             INV_ICM42600_REG_FIFO_COUNTH, 
//             buff, 
//             2);
//         if(ret < 0)return -1;

//         fifo_count = buff[0]<<8|buff[1];

//         if(fifo_count == 0)continue;
//         else{
//             printf("fifo count: %d.\n", buff[0]<<8|buff[1]);
//         }

//         ret = i2c_read_bank_register(
//             inv_icm_42600_i2c_fd,
//             inv_icm_42600_i2c_addr,
//             INV_ICM42600_REG_FIFO_DATA , 
//             fifo_data, 
//             fifo_count);
//         if(ret < 0)return -1;

//     }
// }
