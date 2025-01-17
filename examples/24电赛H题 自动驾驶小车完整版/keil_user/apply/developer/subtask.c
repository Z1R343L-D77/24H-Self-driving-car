#include "headfile.h"
#include "subtask.h"

#define SUBTASK_NUM 20

uint16_t flight_subtask_cnt[SUBTASK_NUM]={0};//任务状态机
void flight_subtask_reset(void)
{
	for(uint16_t i=0;i<SUBTASK_NUM;i++)
	{
		flight_subtask_cnt[i]=0;
	}
}

#define distance_precision_cm 1.0f					
u8 round_grey=0;
//***************************************************任务1***************************************
void auto_line_go(void)
{
	static uint8_t n=10;
	
	switch(flight_subtask_cnt[n])
	{
		case 0:
			if(trackless_output.unlock_flag==UNLOCK)
			{
				ngs_nav_ctrl.update_flag=1;//test
				ngs_nav_ctrl.x=pos.b_x;
				ngs_nav_ctrl.y=pos.b_y;

				gray_break();
				flight_subtask_cnt[n]++;
			}
		break;
		case 1://第一阶段自主寻迹
		
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				position_control();
				turn_ctrl_pwm=steer_gyro_output;
				speed_setup=distance_ctrl.output;
				//期望速度
				speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
				//速度控制
				speed_control_100hz(speed_ctrl_mode);	
				//判断十字路口
				if(gpio_gray_num>0)//经过十字路口
				{
					flight_subtask_cnt[n]++;
					
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯
						trackless_output.unlock_flag=LOCK;	
				}
		break;
		case 2://停止
				speed_expect[0]=speed_expect[1]=0;
				speed_setup=0;
				speed_control_100hz(speed_ctrl_mode);
				speed_error[0]=speed_expect[0]=speed_feedback[0]=speed_integral[0]=0;
				speed_error[1]=speed_expect[1]=speed_feedback[1]=speed_integral[1]=0;
				
		break;
	}
}
static u16 dis_cnt=0;
//****************************************************任务2***************************************
void auto_line_go_2(void)
{
	static uint8_t n=11;
	
	switch(flight_subtask_cnt[n])
	{
		case 0:
			if(trackless_output.unlock_flag==UNLOCK)
			{
				ngs_nav_ctrl.update_flag=1;//test
				ngs_nav_ctrl.x=pos.b_x;
				ngs_nav_ctrl.y=pos.b_y;

				gray_break();
				flight_subtask_cnt[n]++;
			}
		break;
			//
		case 1://第一阶段自主寻迹
		
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				position_control();
				turn_ctrl_pwm=steer_gyro_output;
				speed_setup=distance_ctrl.output;
				//期望速度
				speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
				//速度控制
				speed_control_100hz(speed_ctrl_mode);	
				//判断十字路口
				
				if(gpio_gray_num>0)//经过十字路口
				{
					flight_subtask_cnt[n]++;
						
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯
					
					speed_setup=110;
					
					dis_cnt=smartcar_imu.state_estimation.distance;
				}
		break;
		case 2:
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
				//期望速度
				speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
				//速度控制
				speed_control_100hz(speed_ctrl_mode);	
				//判断十字路口
				if(dis_cnt+121<=smartcar_imu.state_estimation.distance)
				{
					if(gpio_gray_num==0)//第二个圆弧
					{
						flight_subtask_cnt[n]++;
						
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯
						
						ngs_nav_ctrl.update_flag=1;//出弯
						ngs_nav_ctrl.x=pos.d_x;
						ngs_nav_ctrl.y=pos.d_y;

						gray_break();
					}				
				}
		break;
		case 3:
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				position_control();
				turn_ctrl_pwm=steer_gyro_output;
				speed_setup=distance_ctrl.output;
				//期望速度
				speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
				//速度控制
				speed_control_100hz(speed_ctrl_mode);	
				//判断路口
				if(gpio_gray_num>0)//经过十路口
				{
					flight_subtask_cnt[n]++;
					
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯
					
					speed_setup=110;
					dis_cnt=smartcar_imu.state_estimation.distance;
				}
		break;
		case 4:
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
				//期望速度
				speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
				//速度控制
				speed_control_100hz(speed_ctrl_mode);	
				//判断十字路口
				if(dis_cnt+110<=smartcar_imu.state_estimation.distance)
				{
					if(gpio_gray_num==0)//第二个圆弧
					{
						flight_subtask_cnt[n]++;
						
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯
						
						trackless_output.unlock_flag=LOCK;	
					}				
				}
		break;

		case 5:
			
				speed_expect[0]=speed_expect[1]=0;
				speed_setup=0;
				speed_control_100hz(speed_ctrl_mode);
				speed_error[0]=speed_expect[0]=speed_feedback[0]=speed_integral[0]=0;
				speed_error[1]=speed_expect[1]=speed_feedback[1]=speed_integral[1]=0;
				
		break;
	}
}
//****************************************************任务三***************************************
void auto_line_cha_1(void)
{
	static uint8_t n=12;
	static _u8 round_cnt=0;
	switch(flight_subtask_cnt[n])
	{
		case 0:
			if(trackless_output.unlock_flag==UNLOCK)
			{
				ngs_nav_ctrl.update_flag=1;//test
				ngs_nav_ctrl.x=pos.ac_x;
				ngs_nav_ctrl.y=pos.ac_y;

				gray_break();
				flight_subtask_cnt[n]++;
			}
		break;
		//
		case 1://第一阶段自主寻迹
		
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				position_control();
				turn_ctrl_pwm=steer_gyro_output;
				speed_setup=distance_ctrl.output;
				//期望速度
				speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
				//速度控制
			
				speed_control_100hz(speed_ctrl_mode);
				
				//判断路口
				static u16 dis_cnt=0;
				if(gpio_gray_num>0)//经过路口
				{
					round_grey=1;
					flight_subtask_cnt[n]++;
					
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯
						
					gray_status[0]=15;gray_status_backup[0][0]=15;
					
					dis_cnt=smartcar_imu.state_estimation.distance;
				}
		break;
				//
		case 2:
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
				//期望速度
				speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
				//速度控制
				speed_control_100hz(speed_ctrl_mode);	
				//判断十字路口
				if(dis_cnt+20<=smartcar_imu.state_estimation.distance)
				{
					round_grey=0;
					speed_setup=106;
				}
				if(dis_cnt+120<=smartcar_imu.state_estimation.distance)
				{
					if(gpio_gray_num==0)//第二个圆弧
					{

						flight_subtask_cnt[n]++;
						
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯

						gray_break();
						
						ngs_nav_ctrl.update_flag=1;//出弯
						if(round_cnt==0)
						{
							ngs_nav_ctrl.x=pos.bd_x;
							ngs_nav_ctrl.y=pos.bd_y;
						}else
						{
							ngs_nav_ctrl.x=pos.bd_x;
							ngs_nav_ctrl.y=pos.bd_y;
						}

						dis_cnt=smartcar_imu.state_estimation.distance;
					}				
				}
		break;
				//
		case 3:
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				position_control();
				turn_ctrl_pwm=steer_gyro_output;
				speed_setup=distance_ctrl.output;
				//期望速度
				speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		
				
				speed_control_100hz(speed_ctrl_mode);
				//判断路口
				
				static u8 in_round=0;
				if(gpio_gray_num>0)//经过路口
				{
					round_grey=2;
					flight_subtask_cnt[n]++;
					
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯
					
					
					
					gray_status[0]=-15;gray_status_backup[0][0]=-15;
					
					dis_cnt=smartcar_imu.state_estimation.distance;
				}
		break;
			//
		case 4:
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
				//期望速度
				speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
				//速度控制
				speed_control_100hz(speed_ctrl_mode);	
				//判断十字路口
				if(dis_cnt+20<=smartcar_imu.state_estimation.distance)
				{
					round_grey=0;
					speed_setup=96;
						if(round_cnt==3)
							speed_setup=105;
				}


				if(dis_cnt+125<=smartcar_imu.state_estimation.distance)
				{
					if(gpio_gray_num==0)//第二个圆弧
					{
						flight_subtask_cnt[n]=0;//循环
						
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯
						
						smartcar_imu.state_estimation.pos.y=2.5;
						smartcar_imu.state_estimation.pos.x=14;
						
						
						ngs_nav_ctrl.update_flag=1;//出弯
						ngs_nav_ctrl.x=pos.ac_x;
						ngs_nav_ctrl.y=pos.ac_y;
						dis_cnt=smartcar_imu.state_estimation.distance=0;
						gray_break();
						
						round_cnt++;
						if(round_cnt>=1)
						{
							flight_subtask_cnt[n]=5;
							trackless_output.unlock_flag=LOCK;	
						}
					}				
				}
		break;
		//
		case 5:
				speed_expect[0]=speed_expect[1]=0;
				speed_setup=0;
				speed_control_100hz(speed_ctrl_mode);
				speed_error[0]=speed_expect[0]=speed_feedback[0]=speed_integral[0]=0;
				speed_error[1]=speed_expect[1]=speed_feedback[1]=speed_integral[1]=0;
		break;
			
	}
}
//****************************************************任务四***************************************
void auto_line_cha_2(void)
{
	static uint8_t n=13;
	static _u8 round_cnt=0;
	switch(flight_subtask_cnt[n])
	{
		case 0:
			if(trackless_output.unlock_flag==UNLOCK)
			{
				ngs_nav_ctrl.update_flag=1;//test
				ngs_nav_ctrl.x=pos.ac_x;
				ngs_nav_ctrl.y=pos.ac_y;

				gray_break();
				flight_subtask_cnt[n]++;
			}
		break;
		//
		case 1://第一阶段自主寻迹
		
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				position_control();
				turn_ctrl_pwm=steer_gyro_output;
				speed_setup=distance_ctrl.output;
				//期望速度
				speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
				//速度控制
			
				speed_control_100hz(speed_ctrl_mode);
				
				//判断路口
				static u16 dis_cnt=0;
				if(gpio_gray_num>0)//经过路口
				{
					round_grey=1;
					flight_subtask_cnt[n]++;
					
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯
						
					gray_status[0]=15;gray_status_backup[0][0]=15;
					
					dis_cnt=smartcar_imu.state_estimation.distance;
				}
		break;
				//
		case 2:
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
				//期望速度
				speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
				//速度控制
				speed_control_100hz(speed_ctrl_mode);	
				//判断十字路口
				if(dis_cnt+13<=smartcar_imu.state_estimation.distance)
				{
					round_grey=0;
					speed_setup=107;
				}
				if(dis_cnt+120<=smartcar_imu.state_estimation.distance)
				{
					if(gpio_gray_num==0)//第二个圆弧
					{

						flight_subtask_cnt[n]++;
						
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯

						gray_break();
						
						ngs_nav_ctrl.update_flag=1;//出弯
						if(round_cnt==0)
						{
							ngs_nav_ctrl.x=pos.bd_x;
							ngs_nav_ctrl.y=pos.bd_y;
						}else
						{
							ngs_nav_ctrl.x=pos.bd_x;
							ngs_nav_ctrl.y=pos.bd_y;
						}

						dis_cnt=smartcar_imu.state_estimation.distance;
					}				
				}
		break;
				//
		case 3:
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				position_control();
				turn_ctrl_pwm=steer_gyro_output;
				speed_setup=distance_ctrl.output;
				//期望速度
				speed_expect[0]=speed_setup-turn_ctrl_pwm*steer_gyro_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup+turn_ctrl_pwm*steer_gyro_scale;//右边轮子速度期望
		
				
				speed_control_100hz(speed_ctrl_mode);
				//判断路口
				
				static u8 in_round=0;
				if(gpio_gray_num>0)//经过路口
				{
					round_grey=2;
					flight_subtask_cnt[n]++;
					
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯
					
					
					
					gray_status[0]=-15;gray_status_backup[0][0]=-15;
					
					dis_cnt=smartcar_imu.state_estimation.distance;
				}
		break;
			//
		case 4:
				speed_ctrl_mode=1;//速度控制方式为两轮单独控制
				gray_turn_control_200hz(&turn_ctrl_pwm);//基于灰度对管的转向控制
				//期望速度
				speed_expect[0]=speed_setup+turn_ctrl_pwm*turn_scale;//左边轮子速度期望
				speed_expect[1]=speed_setup-turn_ctrl_pwm*turn_scale;//右边轮子速度期望
				//速度控制
				speed_control_100hz(speed_ctrl_mode);	
				//判断十字路口
				if(dis_cnt+18<=smartcar_imu.state_estimation.distance)
				{
					round_grey=0;
					speed_setup=100;
						if(round_cnt==3)
							speed_setup=108;
				}


				if(dis_cnt+125<=smartcar_imu.state_estimation.distance)
				{
					if(gpio_gray_num==0)//第二个圆弧
					{
						flight_subtask_cnt[n]=0;//循环
						
						beep.period=100;//100*5ms
						beep.light_on_percent=0.05f;
						beep.reset=1;	
						beep.times=1;//闪烁1次	
						bling_set(&light_red ,1000,1000,0.5,0,0);//灭红灯 
						
						smartcar_imu.state_estimation.pos.y=2.5;
						smartcar_imu.state_estimation.pos.x=14;
						
						
						ngs_nav_ctrl.update_flag=1;//出弯
						ngs_nav_ctrl.x=pos.ac_x;
						ngs_nav_ctrl.y=pos.ac_y;
						dis_cnt=smartcar_imu.state_estimation.distance=0;
						gray_break();
						
						round_cnt++;
						if(round_cnt>=4)
						{
							flight_subtask_cnt[n]=5;
							trackless_output.unlock_flag=LOCK;	
						}
					}				
				}
		break;
				//
		case 5:
				speed_expect[0]=speed_expect[1]=0;
				speed_setup=0;
				speed_control_100hz(speed_ctrl_mode);
				speed_error[0]=speed_expect[0]=speed_feedback[0]=speed_integral[0]=0;
				speed_error[1]=speed_expect[1]=speed_feedback[1]=speed_integral[1]=0;
		break;
			
	}
}
