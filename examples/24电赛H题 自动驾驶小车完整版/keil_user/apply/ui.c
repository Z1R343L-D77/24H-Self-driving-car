#include "headfile.h"
#include "ui.h"

int16_t page_number=0;

#define LONG_PRESS_MAX  5000
#define Page_Number_Max 32//20

/***************************************************
函数名: void Key_Scan(void)
说明:	按键扫描
****************************************************/
void Key_Scan(void)
{
	if(_button.state[UP].press==SHORT_PRESS)
	{
		page_number--;
		if(page_number<0) page_number=Page_Number_Max-1;
		_button.state[UP].press=NO_PRESS;
		LCD_CLS();
	}
	if(_button.state[DOWN].press==SHORT_PRESS)
	{
		page_number++;
		if(page_number>Page_Number_Max-1) page_number=0;
		_button.state[DOWN].press=NO_PRESS;
		LCD_CLS();
	}
	if(_button.state[ME_3D].press==LONG_PRESS)//中间按键长按为触发
	{
		_button.state[ME_3D].press=NO_PRESS;
		if(trackless_output.unlock_flag==LOCK)	trackless_output.unlock_flag=UNLOCK;//长按后解锁
		else trackless_output.unlock_flag=LOCK;//长按后上锁
		flight_subtask_reset();	//复位sdk子任务状态量
	}
}

/***************************************************
函数名: void screen_display(void)
说明:	屏幕显示与刷新
****************************************************/
void screen_display(void)
{
	Key_Scan();
	static u16 cnt_time_put=0;
	if(++cnt_time_put<10)return;
	cnt_time_put=0;
	
	switch(page_number)
	{
		case 0:
		{
			LCD_clear_L(0,0);display_6_8_string(0,0,"mpu");	
			display_6_8_number(20,0,smartcar_imu.imu_cal_flag);
			display_6_8_number(100,0,trackless_output.unlock_flag);
																																					
			LCD_clear_L(0,2);	display_6_8_string(0,2,"posi");            display_6_8_number_pro(45,2,smartcar_imu.state_estimation.pos.x); display_6_8_number_pro(90,2,smartcar_imu.state_estimation.pos.y);																																
			LCD_clear_L(0,3);	display_6_8_string(0,3,"gray");				display_6_8_number_pro(45,3,gray_status[0]);display_6_8_number_pro(90,3,turn_output);
															
			LCD_clear_L(0,6);	display_6_8_string(0,6,"mode");				display_6_8_number_pro(60,6,sdk_work_mode);
			LCD_clear_L(0,7);write_6_8_number_f1(0,7,smartcar_imu.rpy_deg[0]);
							write_6_8_number_f1(40,7,smartcar_imu.rpy_deg[1]);
							write_6_8_number_f1(80,7,smartcar_imu.rpy_deg[2]);
							
			static uint8_t ver_choose=2;
			display_6_8_string(50,ver_choose+4,"*");
			//通过3D按键来实现可以实现选中的参数行自增加调整
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
				switch(ver_choose)
				{
					case 2:
					{
						sdk_work_mode+=1;
						sdk_work_mode=constrain_float(sdk_work_mode,-32,32);
					}
					break;				
				}
			}

			//通过3D按键来实现可以实现选中的参数行减小调整
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
				switch(ver_choose)
				{
					case 1:
					{
						speed_setup-=1;
						speed_setup=constrain_float(speed_setup,-200,200);
					}
					break;
					case 2:
					{
						sdk_work_mode-=1;
						sdk_work_mode=constrain_float(sdk_work_mode,-32,32);
					}
					break;				
				}
			}
		}
		break;
		//
		case 1:
			LCD_clear_L(0,0);display_6_8_string(0,0,"pos_b_x");display_6_8_number(80,0,pos.b_x);
			LCD_clear_L(0,1);display_6_8_string(0,1,"pos_b_y");display_6_8_number(80,1,pos.b_y);
			LCD_clear_L(0,2);display_6_8_string(0,2,"pos_d_x");display_6_8_number(80,2,pos.d_x);
			LCD_clear_L(0,3);display_6_8_string(0,3,"pos_d_y");display_6_8_number(80,3,pos.d_y);
			LCD_clear_L(0,4);display_6_8_string(0,4,"pos_ac_x");display_6_8_number(80,4,pos.ac_x);
			LCD_clear_L(0,5);display_6_8_string(0,5,"pos_ac_y");display_6_8_number(80,5,pos.ac_y);
			LCD_clear_L(0,6);display_6_8_string(0,6,"pos_bd_x");display_6_8_number(80,6,pos.bd_x);
			LCD_clear_L(0,7);display_6_8_string(0,7,"pos_bd_y");display_6_8_number(80,7,pos.bd_y);
		
			static uint8_t ver_choose=0;
			if(ver_choose==0)		display_6_8_string(70,0,"*");
			else if(ver_choose==1)	display_6_8_string(70,1,"*");
			else if(ver_choose==2)	display_6_8_string(70,2,"*");
			else if(ver_choose==3)	display_6_8_string(70,3,"*");
			else if(ver_choose==4)	display_6_8_string(70,4,"*");
			else if(ver_choose==5)	display_6_8_string(70,5,"*");
			else if(ver_choose==6)	display_6_8_string(70,6,"*");
			else if(ver_choose==7)	display_6_8_string(70,7,"*");
		
			//通过3D按键来实现换行选中待修改参数
			if(_button.state[UP_3D].press==SHORT_PRESS)
			{
					_button.state[UP_3D].press=NO_PRESS;
					ver_choose--;
					if(ver_choose>7) ver_choose=7;	
			}
			if(_button.state[DN_3D].press==SHORT_PRESS)
			{
					_button.state[DN_3D].press=NO_PRESS;
					ver_choose++;
					if(ver_choose>7) ver_choose=0;		
			}	
			
			//通过3D按键来实现可以实现选中的参数行自增加调整
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
				switch(ver_choose)
				{
					case 0:
					{
						pos.b_x+=1;
						WriteFlashParameter(RESERVED_PARAMS_1,pos.b_x,&Trackless_Params);
					}
					break;		
					case 1:
					{
						pos.b_y+=1;
						WriteFlashParameter(RESERVED_PARAMS_2,pos.b_y,&Trackless_Params);
					}
					break;	
					case 2:
					{
						pos.d_x+=1;
						WriteFlashParameter(RESERVED_PARAMS_3,pos.d_x,&Trackless_Params);
					}
					break;		
					case 3:
					{
						pos.d_y+=1;
						WriteFlashParameter(RESERVED_PARAMS_4,pos.d_y,&Trackless_Params);
					}
					break;	
					case 4:
					{
						pos.ac_x+=1;
						WriteFlashParameter(RESERVED_PARAMS_5,pos.ac_x,&Trackless_Params);
					}
					break;		
					case 5:
					{
						pos.ac_y+=1;
						WriteFlashParameter(RESERVED_PARAMS_6,pos.ac_y,&Trackless_Params);
					}
					break;	
					case 6:
					{
						pos.bd_x+=1;
						WriteFlashParameter(RESERVED_PARAMS_7,pos.bd_x,&Trackless_Params);
					}
					break;		
					case 7:
					{
						pos.bd_y+=1;
						WriteFlashParameter(RESERVED_PARAMS_8,pos.bd_y,&Trackless_Params);
					}
					break;						
				}
			}
			
			//通过3D按键来实现可以实现选中的参数行自增加调整
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
				switch(ver_choose)
				{
					case 0:
					{
						pos.b_x-=1;
						WriteFlashParameter(RESERVED_PARAMS_1,pos.b_x,&Trackless_Params);
					}
					break;		
					case 1:
					{
						pos.b_y-=1;
						WriteFlashParameter(RESERVED_PARAMS_2,pos.b_y,&Trackless_Params);
					}
					break;	
					case 2:
					{
						pos.d_x-=1;
						WriteFlashParameter(RESERVED_PARAMS_3,pos.d_x,&Trackless_Params);
					}
					break;		
					case 3:
					{
						pos.d_y-=1;
						WriteFlashParameter(RESERVED_PARAMS_4,pos.d_y,&Trackless_Params);
					}
					break;	
					case 4:
					{
						pos.ac_x-=1;
						WriteFlashParameter(RESERVED_PARAMS_5,pos.ac_x,&Trackless_Params);
					}
					break;		
					case 5:
					{
						pos.ac_y-=1;
						WriteFlashParameter(RESERVED_PARAMS_6,pos.ac_y,&Trackless_Params);
					}
					break;	
					case 6:
					{
						pos.bd_x-=1;
						WriteFlashParameter(RESERVED_PARAMS_7,pos.bd_x,&Trackless_Params);
					}
					break;		
					case 7:
					{
						pos.bd_y-=1;
						WriteFlashParameter(RESERVED_PARAMS_8,pos.bd_y,&Trackless_Params);
					}
					break;						
				}
			}
		
		break;
		//
		default:
		{
			LCD_clear_L(0,0);	display_6_8_string(0,0,"basic");									display_6_8_number(110,0,page_number+1);
		}
	}
}


