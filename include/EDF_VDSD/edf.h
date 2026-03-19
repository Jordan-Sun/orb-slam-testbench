#pragma once

#include <errno.h>
#include <linux/sched.h>
#include <pthread.h>
#include <sched.h>
#include <sys/syscall.h>

enum TaskID {
  SYNC_WITH_IMU_THREAD,
  LOOP_CLOSING_THREAD,
  LOCAL_MAPPING_THREAD,
//   IMU_THREAD,
//   LEFT_CAMERA_THREAD,
//   RIGHT_CAMERA_THREAD
//   DUMMY_THREAD
};

// Ensure all tasks start at the same time for the table to be synchronized
extern struct timespec release_time;

// Multiple reader with no writer so we should be thread safe
const std::vector<std::vector<int>> table_0 = {
    {
        49,
        48,
        47,
        46,
        45,
        44,
    },
    {
        44,
    },
    {
        47,
        44,
    },
};
// Each thread should keep its own indexing to avoid race conditions
