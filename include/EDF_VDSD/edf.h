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

// Multiple reader with no writer so we should be thread safe
const std::vector<std::vector<int>> table_0 = {
    {
        98,
        96,
        94,
        92,
        90,
    },
    {
        99,
        98,
        97,
        96,
        95,
        94,
        93,
        92,
        91,
        90,
    },
    {
        95,
        90,
    },
};
// Each thread should keep its own indexing to avoid race conditions
