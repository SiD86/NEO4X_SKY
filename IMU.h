#ifndef __IMU_H__
#define __IMU_H__

//
// Error bits
//
#define IMU_NO_ERROR					(0x00)
#define IMU_MPU6050_ERROR				(0x01)
#define IMU_BMP280_ERROR				(0x02)

//
// External interface
//
void imu_initialize();
void imu_enable();
void imu_disable();
void imu_process();
void imu_get_position(float* XYZH, float* gyro_XYZ);
bool imu_is_position_updated();
uint32_t imu_get_status();

#endif /* __IMU_H__ */