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

#ifndef SENSOR_F37_CONFIG_H_
#define SENSOR_F37_CONFIG_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "vio/hb_mipi_api.h"
#include "vio/hb_vin_api.h"
#include "x3_sdk_wrap.h"

extern MIPI_SENSOR_INFO_S SENSOR_1LANE_F37_30FPS_10BIT_LINEAR_INFO;
extern MIPI_ATTR_S MIPI_1LANE_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_F37_LINEAR_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_F37_LINEAR_BASE;
extern VIN_DIS_ATTR_S DIS_ATTR_F37_LINEAR_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_F37_LINEAR_BASE;

extern MIPI_SENSOR_INFO_S SENSOR_1LANE_F37_30FPS_10BIT_DOL2_INFO;
extern MIPI_ATTR_S MIPI_1LANE_SENSOR_F37_30FPS_10BIT_DOL2_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_F37_DOL2_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_F37_DOL2_BASE;
extern VIN_DIS_ATTR_S DIS_ATTR_F37_DOL2_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_F37_DOL2_BASE;

extern VIN_DEV_ATTR_S DEV_ATTR_F37_LINEAR_FEEDBACK;
extern VIN_DEV_ATTR_EX_S DEV_ATTR_F37_MD_BASE;

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif // SENSOR_F37_CONFIG_H_