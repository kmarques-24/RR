#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct timespec_t {
    int32_t secs;
    uint32_t nanosecs;
} timespec_t;

timespec_t getTime(void);

#ifdef __cplusplus
}
#endif