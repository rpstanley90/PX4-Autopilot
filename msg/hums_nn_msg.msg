
uint64 timestamp		# time since system start (microseconds)

float32 roll_rate		# Bias corrected angular velocity about X, Y, Z body axis in rad/s
float32 pitch_rate
float32 yaw_rate
float32 roll_rate_sp		# body angular rates in NED frame (setpoint)
float32 pitch_rate_sp		# body angular rates in NED frame (setpoint)
float32 yaw_rate_sp		# body angular rates in NED frame (setpoint)
float32[4] control		# actuator controls (roll, pitch, yaw, thrust)
