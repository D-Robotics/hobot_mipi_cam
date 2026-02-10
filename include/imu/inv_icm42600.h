#ifndef __INV_ICM_42600_H__
#define __INV_ICM_42600_H__

#include "inv_icm42600_reg.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
// #include <cstdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// #define ICM42688_I2C_ADDR 0x68
#define FSYNC_GPIO_CHIP_NUM 4
#define FSYNC_GPIO_CHIP_LINE_OFFSET 22

#define INV_ICM42600_EMPTY_PACKAGE { \
    .temp_origin = 0, \
    .gyro_data_origin = {0}, \
    .accel_data_origin = {0}, \
    .temp_processed = -275.0f, \
    .gyro_data_processed = {0}, \
    .accel_data_processed = {0}, \
    .fsync_success = false, \
    .src_reg = false, \
    .src_fifo = false, \
};

struct inv_icm42600_sensor_state{
    /* DEVICE_CONFIG */
    // uint8_t bank_id;
    uint8_t spi_mode;
    /* DRIVE_CONFIG */
    enum inv_icm42600_slew_rate i2c_slew_rate;
    enum inv_icm42600_slew_rate spi_slew_rate;
    /* FIFO_CONFIG */
    enum inv_icm42600_fifo_mode fifo_mode;
    /* INTF_CONFIG0 */
    bool fifo_hold_last_data_en;
    bool fifo_count_rec;
    bool fifo_count_be;
    bool sensor_data_be;
    enum inv_icm42600_ui_sifs_cfg ui;
    /* PWR_MGMT0 */
    bool temp_sensor_pwr_en;
    enum inv_icm42600_pwr_mgmt0_gyro_mode gyro_pwr_mode;
    enum inv_icm42600_pwr_mgmt0_accel_mode accel_pwr_mode;
    /* GYRO_CONFIG0 */
    enum inv_icm42600_gyro_fs gyro_fs;
    enum inv_icm42600_gyro_odr gyro_odr;
    /* ACCEL_CONFIG0  */
    enum inv_icm42600_accel_fs accel_fs;
    enum inv_icm42600_accel_odr accel_odr;
    /* GYRO_CONFIG1 */
    enum inv_icm42600_temp_filt_bw temp_filt_bw;
    uint8_t gyro_ui_filt_ord;
    uint8_t gyro_dec2_m2_ord;
    /* GYRO_ACCEL_CONFIG0 */
    enum inv_icm42600_LN_filt_bw accel_filt_bw;
    enum inv_icm42600_accel_LP_filt_avg accel_LP_filt_avg;
    enum inv_icm42600_LN_filt_bw gyro_filt_bw;
    /* TMST_CONFIG */
    bool tmst_en;
    bool tmst_fsync_en;
    bool tmst_delta_en;
    bool tmst_res_en;
    bool tmst_to_regs_en;
    /* FIFO_CONFIG1 */
    bool fifo_accel_en;
    bool fifo_gyro_en;
    bool fifo_temp_en;
    bool fifo_tmst_fsync;
    bool fifo_hires_en;
    bool fifo_wm_gt_th;
    bool fifo_resume_partial_rd;
    /* FIFO_CONFIG2 FIFO_CONFIG3 */
    uint16_t fifo_watermark;
    /* FSYNC_CONFIG */
    uint8_t fsync_ui_sel;
    bool fsync_ui_flag_clear;
    bool fsync_polarity;
    /* INTF_CONFIG5(FSYNC_PIN) */
    enum inv_icm42600_pin9_function pin9_func;
    /* INTF_CONFIG1 */
    bool accel_lp_clk_sel;
    bool rtc_mode;
    uint8_t clksel;
};

struct origin_axis_3_data{
    int16_t x, y, z;
    uint64_t fsync_timestamp; /* fsync触发时的系统时间戳 */
    uint16_t deltatime;  /* ODR时间差 */
};

struct processed_axis_3_data{
    float x, y, z;
    uint64_t timestamp; /* ODR时间戳 */
    bool processed;
};

struct inv_icm42600_data_packege{
    /* origin data */
    int16_t temp_origin;
    struct origin_axis_3_data gyro_data_origin;
    struct origin_axis_3_data accel_data_origin;

    /* processed data */
    float temp_processed;
    struct processed_axis_3_data gyro_data_processed;
    struct processed_axis_3_data accel_data_processed;

    /* fsync success */
    bool fsync_success;

    /* source */
    bool src_reg, src_fifo;
};

int inv_icm42600_i2c_fd_auto_init();

int inv_icm42600_all_sel();

int inv_icm42600_pwr_mgmt(
    enum inv_icm42600_pwr_mgmt0_gyro_mode gyro_pwr,
    enum inv_icm42600_pwr_mgmt0_accel_mode accel_pwr,
    bool temp_pwr,
    bool RC_clock_pwr);

int inv_icm42600_filt_setting();

int inv_icm42600_soft_reset();

int inv_icm42600_origin_data_read(
    struct inv_icm42600_data_packege *data_pkt,
    bool try_fsync);

int inv_icm42600_data_process(
    struct inv_icm42600_data_packege *data_pkt);

#ifdef __cplusplus
}
#endif

#endif
