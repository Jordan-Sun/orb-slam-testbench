enum TaskID {
    IMU_THREAD,
    LEFT_IMAGE_THREAD,
    RIGHT_IMAGE_THREAD,
    SYNC_WITH_IMU_THREAD
};

// Multiple reader with no writer so we should be thread safe
const std::vector<std::vector> task_prio_table;
// Each thread should keep its own indexing to avoid race conditions
