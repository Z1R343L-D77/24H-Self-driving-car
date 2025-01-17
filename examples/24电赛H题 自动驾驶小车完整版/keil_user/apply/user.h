#ifndef __USER_H
#define __USER_H
/***********************************************************************************************************************/
#define left_motor_encoder_dir_default   0
#define right_motor_encoder_dir_default  0
#define left_motion_dir_default   0
#define right_motion_dir_default  0
#define tire_radius_cm_default  3.4f			 //轮胎半径，单位为cm
#define pulse_cnt_per_circle_default  1060 //实际轮子转动一周的脉冲数量
#define rangefinder_type_default 0

#define no_voltage_enable_default 0
#define no_voltage_upper_default 11.5f
#define no_voltage_lower_default 10.5f
/***********************************************************************************************************************/
#define speed_expect_default 			50.0f			//自主寻迹时，速度默认设定值
#define work_mode_default   			0 				//小车工作模式出厂默认值
/***********************************************************************************************************************/
#define turn_kp_default1						102.0f 	//基于红外对管识别轨迹时,自主寻迹的位置控制器比例参数KP	
#define turn_ki_default1						0.0f		//基于红外对管识别轨迹时,自主寻迹的位置控制器积分参数KI	
#define turn_kd_default1						220			//基于红外对管识别轨迹时,自主寻迹的位置控制器微分参数KD	
#define turn_scale_default        				0.15f		//转向控制器输出转换到轮子期望差速时的量程转换系数
/***********************************************************************************************************************/
#define steer_gyro_kp_default  		1.5f			//转向陀螺仪控制时，角速度内环比例系数KP
#define steer_gyro_ki_default  		0.0f			//转向陀螺仪控制时，角速度内环积分系数KI
#define steer_gyro_kd_default  		3.0f			//转向陀螺仪控制时，角速度内环微分系数KD	
#define	steer_gyro_scale_default	0.5f			//转向陀螺仪控制时，角速度内环控制器输出转换到轮子期望差速时的量程转换系数
/***********************************************************************************************************************/
#define speed_kp_default  				8.0f			//速度控制器比例参数KP
#define	speed_ki_default				0.6f			//速度控制器积分参数KI
#define	speed_kd_default				0.0f			//速度控制器微分参数KD

#endif




