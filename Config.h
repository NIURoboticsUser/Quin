
#define ARM_ON_LOAD // If this is defined, the quadcopter will be armed when the Arduino boots.


// PID settings
#define PID_PITCH_P 0.1
#define PID_PITCH_I 0.0
#define PID_PITCH_D 0.0
#define PID_PITCH_IMAX 5.0

#define PID_ROLL_P 0.12
#define PID_ROLL_I 0.0
#define PID_ROLL_D 0.0
#define PID_ROLL_IMAX 5.0

#define PID_YAW_P 0.075
#define PID_YAW_I 0.0
#define PID_YAW_D 0.0
#define PID_YAW_IMAX 5.0

#define PID_ALT_P 0.0
#define PID_ALT_I 0.0
#define PID_ALT_D 0.0
#define PID_ALT_IMAX 5.0

#define PID_PITCH_SCALAR 10
#define PID_ROLL_SCALAR 10
#define PID_YAW_SCALAR 1

#define PID_STABLE_SCALAR ((25.0)/340)
