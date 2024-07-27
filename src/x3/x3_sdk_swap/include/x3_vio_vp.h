/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 D-Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_VP_H_
#define X3_VIO_VP_H_

#include "x3_sdk_wrap.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
int x3_vp_init();
int x3_vp_alloc(vp_param_t *param);
int x3_vp_free(vp_param_t *param);
int x3_vp_deinit();

#ifdef __cplusplus
};
#endif /* __cplusplus */

#endif // X3_VIO_VP_H_