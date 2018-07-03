#ifndef IMU_H_
#define IMU_H_

//
// Error bits
//
#define IMU_NO_ERROR					(0x00)
#define IMU_MPU6050_ERROR				(0x01)
#define IMU_BMP280_ERROR				(0x02)

//
// External interface
//
extern void imu_initialize(void);
extern void imu_enable(void);
extern void imu_disable(void);
extern void imu_process(void);
extern void imu_get_position(float* XYZH, float* gyro_XYZ);
extern bool imu_is_position_updated(void);
extern uint32_t imu_get_status(void);

#endif /* IMU_H_ */