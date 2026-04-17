#pragma once

#include "BNO08x.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

typedef struct uros_service_params_t {
    bno08x_euler_angle_t euler;
} uros_service_params_t;

BaseType_t uros_service(void);

#ifdef __cplusplus
}
#endif