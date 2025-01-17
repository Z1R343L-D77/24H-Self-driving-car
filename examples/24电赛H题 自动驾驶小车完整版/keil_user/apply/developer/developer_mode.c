#include "headfile.h"
#include "sdk.h"
#include "subtask.h"
#include "user.h"
#include "developer_mode.h"

position pos;

int16_t sdk_work_mode=0; 

void sdk_duty_run(void)
{
	if(trackless_output.init==0)
	{		
		trackless_output.yaw_ctrl_mode=ROTATE;
		trackless_output.yaw_outer_control_output=0;
		trackless_output.init=1;
		flight_subtask_reset();//复位sdk子任务状态量
	}
	
	switch(sdk_work_mode)
	{	
		case 0://任务1-2坐标确定
		{	
			smartcar_imu.state_estimation.pos.y=smartcar_imu.state_estimation.pos.x=0;
		}
		break;
		case 1://任务1
		{
			auto_line_go();//直走
		}
		break;
		//
		case 2://任务2
		{
			auto_line_go_2();//第二个圆弧
		}
		break;
		//
		case 3://任务3-4坐标确定
				smartcar_imu.state_estimation.pos.y=smartcar_imu.state_estimation.pos.x=0;
		break;
		//
		case 4://任务3
			auto_line_cha_1();
		break;
		//
		case 5://任务4
			auto_line_cha_2();
		break;
		//
		case 6:
				
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				position_control();
				turn_ctrl_pwm=steer_gyro_output;
				speed_setup=distance_ctrl.output;
				//期望速度
				speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
				//速度控制
				speed_control_100hz(speed_ctrl_mode);
		break;
		//
		default:
			trackless_output.unlock_flag=LOCK;//长按后上锁
			flight_subtask_reset();	//复位sdk子任务状态量
			smartcar_imu.state_estimation.pos.y=smartcar_imu.state_estimation.pos.x=0;
		break;
	}
}

