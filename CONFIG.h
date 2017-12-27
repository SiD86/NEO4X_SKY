#pragma once
// **************** ORIENTATION SUBSYSTEM **************** //
#define OSS_MPU6050_DATA_RDY_PIN					(5)

// **************** FLY CONTROL SUBSYSTEM **************** //
// Range PID control level
#define FCS_PID_ENABLE_THRUST_THRESHOLD				(0)    
#define FCS_PID_FREQUENCY							(2500)	// us
#define FCS_PID_MIN_LEVEL							(-300)
#define FCS_PID_MAX_LEVEL							(+300)
