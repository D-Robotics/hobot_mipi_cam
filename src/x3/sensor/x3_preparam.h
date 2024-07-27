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

#ifndef X3_PREPARAM_H
#define X3_PREPARAM_H

#include "x3_sdk_wrap.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
int f37_linear_vin_param_init(x3_vin_info_t* vin_info);
int f37_dol2_vin_param_init(x3_vin_info_t* vin_info);
int imx415_linear_vin_param_init(x3_vin_info_t* vin_info);
int imx586_linear_vin_param_init(x3_vin_info_t* vin_info);
int gc4c33_linear_vin_param_init(x3_vin_info_t* vin_info);
int gc4663_linear_vin_param_init(x3_vin_info_t* vin_info);
int imx219_linear_vin_param_init(x3_vin_info_t* vin_info);
int imx477_linear_vin_param_init(x3_vin_info_t* vin_info);
int ov5647_linear_vin_param_init(x3_vin_info_t* vin_info);
int vps_grp_param_init(x3_vps_info_t *vps_info, int width, int height);
int vps_chn_param_init(x3_vps_chn_attr_t *vps_chn_attr, int chn_id, int width, int height, int fps);

#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif // X3_PREPARAM_H

