#ifndef __DATATYPE_H
#define __DATATYPE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef   signed          char int8;
typedef unsigned          char _u8;
typedef unsigned          char u8;
typedef unsigned          char uint8;
typedef unsigned          char byte;
typedef   signed short    int int16;
typedef unsigned short    int uint16;
typedef unsigned short    int _u16;
typedef unsigned short    int u16;
typedef unsigned long     int _u32; 
typedef unsigned long     int u32; 

enum 
{
	_ROL=0,
	_PIT,
	_YAW
};

enum 
{
	ROL=0,
	PIT,
	YAW
};

typedef struct
{
  float x;
  float y;
  float z;
}vector3f;

typedef struct
{
  float x;
  float y;
}vector2f;


typedef struct
{
	uint8_t bit1	:1;
	uint8_t bit2	:1;
	uint8_t bit3	:1;
	uint8_t bit4	:1;
	uint8_t bit5	:1;
	uint8_t bit6	:1;
	uint8_t bit7	:1;
	uint8_t bit8	:1;
	uint8_t bit9	:1;
	uint8_t bit10	:1;
	uint8_t bit11	:1;
	uint8_t bit12	:1;
	uint8_t bit13	:1;
	uint8_t bit14	:1;
	uint8_t bit15	:1;
	uint8_t bit16	:1;
}gray_flags;


typedef struct
{
  volatile float last_time;
  volatile float current_time;
  volatile float period;
  volatile uint16_t period_int;//单位ms
}systime;


typedef enum
{
	WHO_AM_I_MPU6050  =0x68,
}IMU_ID_READ;


typedef struct
{
	int8_t left_encoder_dir_config,right_encoder_dir_config;//编码器方向配置
	int8_t left_motion_dir_config	,right_motion_dir_config; //电机运动方向配置
	float wheel_radius_cm;				//轮胎半径,单位为cm
	uint16_t pulse_num_per_circle;//轮胎转动一圈累计的脉冲数量
	uint16_t servo_median_value1,servo_median_value2;
}motor_config;

typedef struct
{
	int16_t left_motor_cnt,right_motor_cnt;//单个采样周期内的脉冲数量
	int8_t left_motor_dir,right_motor_dir; //运动方向
	float left_motor_speed_rpm,right_motor_speed_rpm;//转速单位转每分钟
	float left_motor_gyro_rps,right_motor_gyro_rps;//转速单位rad/s
	float left_motor_speed_cmps,right_motor_speed_cmps;//转速c单位为cm/s
	float left_motor_period_ms,right_motor_period_ms;
	
	int32_t left_motor_total_cnt,right_motor_total_cnt;
	int32_t left_motor_period_cnt,right_motor_period_cnt;
	uint8_t left_motor_cnt_clear,right_motor_cnt_clear;
	
}encoder;

typedef struct
{
	float value;
	uint8_t enable;
	float upper;
	float lower;
	uint16_t low_vbat_cnt;
}low_voltage;


typedef struct
{
	float speed;
	float azimuth;
	float w;
	vector2f pos;
	vector2f vel;
	float distance;
}two_wheel_model;



typedef struct
{
	// 传感器原始数据
	vector3f _gyro_dps_raw,gyro_dps_raw;
	vector3f _accel_g_raw,accel_g_raw;
	vector3f mag_tesla_raw;
	vector3f last_mag_raw;
	float temperature_raw,last_temperature_raw;
	float temperature_filter;
	float vbat;
	// 校准后的传感器数据
	vector3f gyro_dps;
	vector3f accel_g;
	vector3f mag_tesla;
	
	// 传感器偏移和尺度校正参数
	vector3f gyro_offset;
	vector3f accel_scale,accel_offset;
	
	// 电机速度相关数据
	float left_motor_speed_cmps;
	float right_motor_speed_cmps;
	float average_speed_cmps;
	
	// 四元数初始化状态和数据
	uint8_t quaternion_init_ok;
	float quaternion_init[4];//初始四元数
	float quaternion[4];//四元数

	// 姿态角和角速度数据
	float rpy_deg[3];
	float rpy_gyro_dps[3];
	float rpy_gyro_dps_enu[3];
	vector3f accel_earth_cmpss;
	vector2f accel_body_cmpss;
	float sin_rpy[3];
	float cos_rpy[3];
	float cb2n[9];

	// 姿态观测和卡尔曼滤波结果
	float rpy_obs_deg[3];//观测姿态角度
	float rpy_kalman_deg[3];

	// 方位角数据
	float yaw_gyro_enu;

	// IMU收敛状态和标志
	uint16_t imu_convergence_cnt;
	uint8_t imu_convergence_flag;
	uint8_t temperature_stable_flag;
	uint8_t imu_cal_flag;
	uint8_t imu_health;
	uint8_t lpf_init;
	
	// 状态估计模型
	two_wheel_model state_estimation;
}sensor;

typedef enum
{
	LOCK 		=0x00,
	UNLOCK  	=0x01,
}LOCK_STATE;

#define ABS(X)  (((X)>0)?(X):-(X))
#define MAX(a,b)  ((a)>(b)?(a):(b))
#define MIN(a,b)  ((a)>(b)?(b):(a))

typedef struct{
	float x;												//x方向位置/速度期望
	float y;												//y方向位置/速度期望
	float z;												//z方向位置/偏航角速度期望
	
	uint8_t update_flag;									//指令更新标志位
	uint8_t ctrl_finish_flag;								//控制完毕标志位
	
	uint16_t cnt; 											//位置控制完毕判断计数器
	float dis_cm;											//总位置偏差
}nav_ctrl;


#endif


