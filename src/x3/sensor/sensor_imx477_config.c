// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sensor_imx477_config.h"

MIPI_SENSOR_INFO_S SENSOR_2LANE_IMX477_50FPS_12BIT_LINEAR_INFO = {
	.deseEnable = 0, // 该 sensor 是否有 serdes（串解器）
	.inputMode = INPUT_MODE_MIPI, // sensor 接入方式,mipi还是dvp
	.sensorInfo = {
		.port = 0, // sensor的逻辑编号，必须从0开始
		.dev_port = 0, // 每路 sensor 操作的驱动节点，一个驱动支持多个节点。 snsinfo 中的dev_port 必须等于pipeId，多目摄像头设置的时候需要特别注意
		.bus_type = 0, // 访问总线类型， 0 是 i2c,1 是 spi
		.bus_num = 3, // 总线号，根据具体板子硬件原理图确定 , 不配置默认 i2c5
		.fps = 30, // 帧率，用来选择使用哪一组帧率的sensor参数
		.resolution = 1080, // sensor 行宽, 必须要和mipi属性配置一致
		.sensor_addr = 0x1a, // sensor i2c 设备地址
		.entry_index = 0, // sensor 使用的 mipi 索引, 0~3，对应mipi host的序号
		.sensor_mode = NORMAL_M, // sensor 工作模式， 1 是 normal,2 是dol2,3 是 dol3
		.reg_width = 16, // sensor 寄存器地址宽度
		.sensor_name = "imx477", // sensor的名字，在libcam.so中会根据这个名字组合出 libf37.so
							// 这个库文件，然后调用dlopen运行时打开sensor库， libf37.so 这个库的生成
							// 请参考sensor点亮文档，以f37 为例是 f37_utility.c f37_utility.h
							// 请放到sdk包的hbre/camera/utility/sensor目录下make生成libf37.so文件
	}
};

MIPI_ATTR_S MIPI_2LANE_SENSOR_IMX477_50FPS_12BIT_LINEAR_ATTR = {
    .mipi_host_cfg = {
        .lane = 2, // 硬件上sensor用了几个mipi数据lane，f37用的1 lane, os8a10 4K的用了4 lane
        .datatype = 0x2C, // sensor的输出数据类型，请参考 《X3J3平台AIOT媒体系统接口手册.pdf》第3.5.36节 DATA TYPE
        .mclk = 2400, // mipi 模块主时钟，目前默认是24MHz
        .mipiclk = 2250, // sensor 输出总的 mipibit rate, 单位Mbits/秒
        .fps = 50, // sensor 输出实际帧率, 和 MIPI_SENSOR_INFO_S 是什么关系？
        .width = 1920, // sensor实际输出的分辨率，一般根据sensor供应商的配置参数配置就行
        .height = 1080,
        .linelenth = 2976, // sensor 输出带 blanking 的总行长， 一般根据sensor供应商的配置参数配置就行
        .framelenth = 1313, // sensor 输出带 blanking的总行数
        .settle = 30, // sensor 输出实际Ttx-zero + Ttx-prepare时间（ clk 为单位）, 一般0-100之间尝试
		.channel_num = 1, // 使用虚通道的个数, linear 用一个，dol2 用 2 个， dol3 用3个
		.channel_sel = {0} // 保存每个虚通道的值，在调用HB_VIN_SetDevVCNumber接口时用第一个，调用HB_VIN_AddDevVCNumber用后面的2 3 4的配置
    },
    .dev_enable = 0 // mipi dev 是否使能， 1是使能， 0 是关闭, 只用开启mipi bypass的开启
};

VIN_DEV_ATTR_S DEV_ATTR_IMX477_LINEAR_BASE = {
    .stSize = { // 输入的数据
		.format = 0, // 像素格式， format 为 0 代表是raw, 根据 pixel_lenght 来表示究竟是 raw8, raw12 还是 raw16 。
		.width = 1920, // 数据宽
		.height = 1080, // 数据高
		.pix_length = 2 // format=0表示raw    pix_length= 0-raw8   1-raw10  2-raw12
    },
    .mipiAttr = { // sif(dev) 输入的接口模式， mipi or dvp,目前都是 mipi
        .enable = 1, // mipi 使能 ,0 是关闭， 1 是使能
        .ipi_channels = 1, // sif和isp之间的硬件通道，可以把多个mipi虚拟通道合并到这一个ipi channel中，ipi_channels 表示用了几个 channel ，默认是 0 开始，如果设置是 2 ，是用了 0 ，1
        .ipi_mode = 1, // 当 DOL2 分成两路 linear 或者 DOL3 分成一路 DOl2 和一路 linear 或者三路linear 的时候，此值就赋值为 2 或 3.
        .enable_mux_out = 1, // 未用
        .enable_frame_id = 1, // 是否使能 frameid, 是否做智能的时候要用？
        .enable_bypass = 0, // 使能mipi host到mipi dev的bypass
        .enable_line_shift = 0, // 未用
        .enable_id_decoder = 0, // 未用
        .set_init_frame_id = 1, // 初始 frame id 值一般为 1
        .set_line_shift_count = 0, // 未用
        .set_bypass_channels = 1, // 未用
        .enable_pattern = 0, // 是否使能 testpartern
    },
    .DdrIspAttr = { // isp(pipe) 的输入属性配置， offline 或者是回灌
        .buf_num = 4, // 回灌的存储数据的 buf 数目
        .raw_feedback_en = 0, // 使能回灌模式，不能和 offline 模式同时开启，独立使用
        .data = { // 数据格式同 VIN_DEV_SIZE_S
            .format = 0,
            .width = 1920,
            .height = 1080,
            .pix_length = 2,
        }
    },
    .outDdrAttr = { // sif(dev) 的输出到 ddr 配置
        .stride = 2880, // 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来（而且结果要32对齐），8bit：x1, 10bit: x1.25 12bit: x1.5,例F37 raw10，1920 x 1.25 = 2400
        .buffer_num = 8, // dev 输出到 ddr 的 buf 个数，最少可设置为6
        //.frameDepth = 1, // 默认不配置最多 get 的帧数 , buffer_num 是总 buff数量，建议 frameDepth 值最大是ddrOutBufNum – 4
    },
    .outIspAttr = { // sif 到 isp 一些属性设置
        .dol_exp_num = 1, // 曝光模式， 1 为普通模式， dol 2 或者 3设置对应数目
        .enable_dgain = 0, // ISP 内部调试参数，暂可忽略
        .set_dgain_short = 0, // ISP 内部调试参数，暂可忽略
        .set_dgain_medium = 0, // ISP 内部调试参数，暂可忽略
        .set_dgain_long = 0, // ISP 内部调试参数，暂可忽略
        .vc_short_seq = 0, // 用来描述 DOL2/3 模式下，短帧的顺序
        .vc_medium_seq = 0, // 用来描述 DOL2/3 模式下，普通帧的顺序
        .vc_long_seq = 0, // 用来描述 DOL2/3 模式下，长帧的顺序
    }
};

VIN_PIPE_ATTR_S PIPE_ATTR_IMX477_LINEAR_BASE = {
	.ddrOutBufNum = 6, // ISP->GDC ddr的buf数量，最少可以设置成 2
	.snsMode = SENSOR_NORMAL_MODE, // sensor 工作模式，要和MIPI_SENSOR_INFO_S配置的sensor_mode一致
	.stSize = { // 数据格式同 VIN_DEV_SIZE_S
		.format = 0,
		.width = 1920,
		.height = 1080,
	},
	.cfaPattern = PIPE_BAYER_RGGB, // 数据格式布局(sensor输出的图像格式是RGGB还是BGGR),和sensor保持一致，查sensor数据手册可知
	.temperMode = 0, // 时序降噪模式，2代表2帧融合，3代表3帧融合
	.ispBypassEn = 0, // 是否使能 isp 的 bypass, sensor必须支持输出yuv
	.ispAlgoState = 1, // 是否启动 3a 算法库 ,1 是启动， 0 是关闭
	.bitwidth = 12, // raw图位宽，有效值 8 、 10 、 12 、14 、 16 、 20
	.calib = { // 是否开启 sensor 矫正数据加载，1 是开启，0 是关闭。ISP使用这些数据
		.mode = 1, // 是否开启 sensor 矫正数据加载
		.lname = "libimx477_linear.so", // 对应使用的校准库
	}
};

VIN_DIS_ATTR_S DIS_ATTR_IMX477_LINEAR_BASE = {
	.picSize = { // 输入数据宽高
		.pic_w = 1919, // 需要设置比接入尺寸 -1 的 size, 如果 ISP 输出1920 , 则这里设置 1919
		.pic_h = 1079, // 需要设置比接入尺寸 -1 的 size
	},
	.disPath = { // 输出类型
		.rg_dis_enable = 0, // 是否开启
		.rg_dis_path_sel = 1, // 决定进dis的数据来源，0代表ISP输出给dis，1代表LDC输出给dis
	},
	.disHratio = 65536, // 设置为 65536, 这个值是内存位置，需要调试出来
	.disVratio = 65536, // 设置为 65536, 这个值是内存位置，需要调试出来
	.xCrop = { // 处理区域修正
		.rg_dis_start = 0,
		.rg_dis_end = 1919,
	},
	.yCrop = { // 处理区域修正
		.rg_dis_start = 0,
		.rg_dis_end = 1079,
	},
	.disBufNum = 8,
};

VIN_LDC_ATTR_S LDC_ATTR_IMX477_LINEAR_BASE = {
  .ldcEnable = 0, // LDC 是否使能
  .ldcPath = { // 输出类型
	 .rg_y_only = 0, // ???
	 .rg_uv_mode = 0, // ???
	 .rg_uv_interpo = 0, // ???
	 .rg_h_blank_cyc = 32, // tunning 用
  },
  .yStartAddr = 524288, // Iram 使用地址的内存地址
  .cStartAddr = 786432, // Iram 使用地址的内存地址
  .picSize = { // 输入的尺寸
	 .pic_w = 1919, // 需要设置比接入尺寸 -1 的 size, 如果 ISP 输出 1920 ,则这里设置 1919
	 .pic_h = 1079,
  },
  .lineBuf = 99, // 值设置 99, 为什么？
  .xParam = { // 参数需要 tuning
	 .rg_algo_param_b = 1,
	 .rg_algo_param_a = 1,
  },
  .yParam = { // 参数需要 tuning
	 .rg_algo_param_b = 1,
	 .rg_algo_param_a = 1,
  },
  .offShift = { // 处理区域修正
	 .rg_center_xoff = 0,
	 .rg_center_yoff = 0,
  },
  .xWoi = { // 处理区域修正
	 .rg_start = 0,
	 .rg_length = 1919,
  },
  .yWoi = { // 处理区域修正
	 .rg_start = 0,
	 .rg_length = 1079,
  }
};
