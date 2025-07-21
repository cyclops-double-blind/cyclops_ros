InitializationFailureVisionDigest[] vision_solutions
InitializationFailureIMUDigest[] imu_solutions

int32 failure_reason
int32 UNKNOWN = 0
int32 VISION_INITIALIZATION_FAILED = 1
int32 NO_IMU_MATCH_CANDIDATE = 2
int32 AMBIGUOUS_IMU_MATCH = 3
int32 UNACCEPTABLE_VISION_SOLUTION = 4
int32 UNACCEPTABLE_IMU_SOLUTION = 5
int32 AMBIGUOUS_VISION_SOLUTION = 6

string failure_reason_readable
