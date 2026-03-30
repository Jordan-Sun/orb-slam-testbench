#pragma once

#include <atomic>
#include <errno.h>
#include <linux/sched.h>
#include <pthread.h>
#include <sched.h>
#include <sys/syscall.h>
#include <vector>

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
extern std::atomic<bool> fallback_flag;

// Multiple reader with no writer so we should be thread safe
const std::vector<std::vector<int>> table_switching_deadline = {
	{ 49, 43, 38, 32, },
	{ },
	{ },
};
const std::vector<std::vector<int>> table_0 = {
	{ 46, 40, 34, 28, },
	{ 48, 47, 45, 42, 41, 39, 37, 35, 33, 31, 30, 27, },
	{ 44, 36, 29, },
};
const std::vector<std::vector<int>> table_1 = {
	{ 47, 44, 41, 38, },
	{ 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, },
	{ 46, 42, 38, },
};


#define EDF_SWITCHING_TABLE (!fallback_flag.load() ? table_switching_deadline : table_1)
#define EDF_ACTIVE_TABLE (!fallback_flag.load() ? table_0 : table_1)

// Each thread should keep its own indexing to avoid race conditions
