#include "BNO08x.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <fcntl.h>    // for open(), O_WRONLY, O_CREAT, etc.
#include <unistd.h>   // for close(), write(), etc.

void init_imu();
BaseType_t imu_service(void);

typedef struct {
    struct {float w, x, y, z;} quat;        // Orientation (quat)
    struct {float roll, pitch, yaw;} euler; // Orientation (euler)
    struct {float x, y, z;} lin_accel;      // Acceleration (linear)
    struct {float x, y, z;} ang_vel;        // Velocity (angular)
    struct {float x, y, z;} mag;            // Magnetometer
    uint32_t timestamp;
} imu_data_t;

