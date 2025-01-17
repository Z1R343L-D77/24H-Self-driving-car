#ifndef __SDK_H
#define __SDK_H


typedef enum
{
	ROTATE=0,							//手动偏航控制模式
  AZIMUTH=1,						//绝对偏航角度控制模式
	CLOCKWISE=2,					//相对偏航角度顺时针控制模式	
	ANTI_CLOCKWISE=3,			//相对偏航角度逆时针控制模式	
	CLOCKWISE_TURN=4,			//角速度控制顺时针模式
	ANTI_CLOCKWISE_TURN=5,//角速度控制逆时针模式
}YAW_CTRL_MODE;

typedef enum
{
	MOTOR1=0,
	MOTOR2,
	MOTOR3,
	MOTOR4,
	MOTOR5,
	MOTOR6,
	MOTOR7,
	MOTOR8,
	MOTOR_NUM
}motor;


typedef struct
{
	float roll_outer_control_output;  	//横滚姿态控制器输入
	float pitch_outer_control_output; 	//俯仰姿态控制器输入
	float yaw_outer_control_output;	  	//偏航姿态控制器输入

	uint16_t yaw_ctrl_cnt;							//偏航控制计数器
	YAW_CTRL_MODE yaw_ctrl_mode;					//偏航控制模式
	uint8_t yaw_ctrl_start;							//偏航控制开始标志位
	uint8_t yaw_ctrl_end;							//偏航控制结束标志位
	uint32_t start_time_ms;							//偏航控制开始时间
	uint32_t execution_time_ms;						//偏航控制执行时间
	uint8_t init;									//偏航控制初始化标志位

	uint8_t roll_pitch_angle_limit_enable;
	LOCK_STATE unlock_flag;
}controller_output;

void ctrl_params_init(void);

void position_control(void);
void distance_control(void);	

extern float steer_gyro_scale;
extern float	steer_gyro_expect,steer_gyro_output;

extern controller steerangle_ctrl,steergyro_ctrl;
extern controller distance_ctrl,azimuth_ctrl;
extern controller_output trackless_output;

extern nav_ctrl ngs_nav_ctrl;

void e2prom_init(void);
#endif


