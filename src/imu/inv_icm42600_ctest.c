#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "inv_icm42600.h"

int main(void){
    int ret;

    ret = inv_icm42600_i2c_fd_auto_init();
    printf("[inv_icm42600_i2c_fd_auto_init]run return: %d.\n",ret);

    ret = inv_icm42600_soft_reset();
    printf("[inv_icm42600_soft_reset]run return: %d.\n",ret);

    ret = inv_icm42600_all_sel();
    printf("[inv_icm42600_all_sel]run return: %d.\n",ret);

    ret = inv_icm42600_pwr_mgmt(
        INV_ICM_42600_GYRO_LN,
        INV_ICM_42600_ACCEL_LN,
        true,
        true);
    printf("[inv_icm42600_pwr_mgmt]run return: %d.\n",ret);

    ret = inv_icm42600_filt_setting();
    printf("[inv_icm42600_filt_setting]run return: %d.\n",ret);

    usleep(1000000);

    while(1){
        struct inv_icm42600_data_packege dtpkt = INV_ICM42600_EMPTY_PACKAGE;

        ret = inv_icm42600_origin_data_read(&dtpkt, true);
        
        system("clear");
        for(int i=0; i<50; i++)printf("=");printf("\n");

        printf("[origin data read]function return: %d\n", ret);

        ret = inv_icm42600_data_process(&dtpkt);

        printf("[data process]function return: %d\n", ret);

        printf("[fsync test]");
        if(dtpkt.fsync_success)printf("Successed.\n");
        else printf("Failed.\n");

        printf("[temp_data]%.3f unit:Celsius\n", dtpkt.temp_processed);
        printf("[accel_data] x:%8.3f    y:%8.3f    z:%8.3f    unit:g\n", dtpkt.accel_data_processed.x, dtpkt.accel_data_processed.y, dtpkt.accel_data_processed.z);
        printf("[gyro_data]  x:%8.3f    y:%8.3f    z:%8.3f    unit:dps\n", dtpkt.gyro_data_processed.x, dtpkt.gyro_data_processed.y, dtpkt.gyro_data_processed.z);
        printf("[timestamp_data]%ld\n", dtpkt.accel_data_processed.timestamp);

        for(int i=0; i<50; i++)printf("=");printf("\n");

        usleep(100000);
    }

    return 0;
}

