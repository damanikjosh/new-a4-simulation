float64 latitude_deg
float64 longitude_deg
float32 relative_altitude_m
float32 speed_m_s
bool is_fly_through
float32 gimbal_pitch_deg
float32 gimbal_yaw_deg

uint8 CAMERA_ACTION_NONE = 0
uint8 CAMERA_ACTION_TAKE_PHOTO = 1
uint8 CAMERA_ACTION_START_PHOTO_INTERVAL = 2
uint8 CAMERA_ACTION_STOP_PHOTO_INTERVAL = 3
uint8 CAMERA_ACTION_START_VIDEO = 4
uint8 CAMERA_ACTION_STOP_VIDEO = 5
uint8 CAMERA_ACTION_START_PHOTO_DISTANCE = 6
uint8 CAMERA_ACTION_STOP_PHOTO_DISTANCE = 7

uint8 camera_action
float32 loiter_time_s
float64 camera_photo_interval_s
float32 acceptance_radius_m
float32 yaw_deg
float32 camera_photo_distance_m