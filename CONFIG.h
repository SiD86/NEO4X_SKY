#pragma once
// **************** ORIENTATION SUBSYSTEM **************** //
#define OSS_MPU6050_DATA_RDY_PIN					(5)

// **************** FLY CONTROL SUBSYSTEM **************** //
// Range PID control level
#define PID_ENABLE_THRUST_THRESHOLD				(0)    
#define PID_FREQUENCY							(5000)	// us
#define PID_MIN									(-300)
#define PID_MAX									(+300)
