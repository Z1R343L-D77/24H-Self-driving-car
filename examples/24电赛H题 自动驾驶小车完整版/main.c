/****************************************************************************************
	电机控制MPWM	
		PA4-A0-PWM-CH3	  右边电机调速INA1
		PA7-->A0-PWM-CH2	右边电机调速INA2
		PA3-->A0-PWM-CH1	左边电机调速INB1
		PB14-->A0-PWM-CH0	左边电机调速INB2		
	编码器测速ENC	
		PB4-RIGHT-PULSE	  右边电机脉冲倍频输出P1
		PB5-->LEFT-PULSE	左边电机脉冲倍频输出P2
		PB6-->RIGHT-DIR	  右边电机脉冲鉴相输出D1
		PB7-->LEFT-DIR	  左边电机脉冲鉴相输出D2
	外置IMU接口IMU	
		PA29-I2C-SCL	MPU6050-SCL
		PA30-->I2C-SDA	MPU6050-SDA
	电池电压采集	
		PA26-ADC-VBAT	需要外部分压后才允许接入
****************************************************************************************/
#include "ti_msp_dl_config.h"
#include "headfile.h"

void maple_duty_200hz(void);
void duty_1000hz(void);

int main(void)
{
	usart_irq_config();             //串口中断配置
	SYSCFG_DL_init();	         	//系统资源配置初始化	
	OLED_Init();					//显示屏初始化
	nADC_Init();					//ADC初始化
	nGPIO_Init();					//蜂鸣器初始化
	w25qxx_gpio_init();     		//板载w25q64初始化
	e2prom_init();					//e2prom初始化
	ctrl_params_init();				//控制参数初始化
	ICM206xx_Init();				//加速度计/陀螺仪初始化	
	Encoder_Init();					//编码器资源初始化
	Button_Init();					//板载按键初始化
	timer_irq_config();
	while(1)
	{
		maple_duty_200hz();
	}
}
/***************************************
函数名:	void duty_200hz(void)
说明: 200hz主函数任务函数
***************************************/
void maple_duty_200hz(void)
{
	if(!f_5ms)return;
	f_5ms=0;
	if(!low_voltage_flag)
		screen_display();//屏幕显示
	
	laser_light_work(&beep);       				//电源板蜂鸣器驱动
	bling_working(0);							//RGB灯状态机
	read_button_state_all();       				//按键状态读取
	
	// static u16 cnt_time_put=0;
	// if(++cnt_time_put>=4)
	// {
	// 	cnt_time_put=0;
	// 	printf("%.1f,%.1f,%.1f \n",seektrack_ctrl[0].kp,seektrack_ctrl[0].kd,speed_setup);
	// }
	
	static u16 cnt_100ms=0;
	if(++cnt_100ms>=20)
	{
		cnt_100ms=0;	
		adc_statemachine();							//adc采集状态机
		battery_voltage_detection();	 			//电池电压检测
		get_battery_voltage();				 		//ADC数据获取
	}
}
/***************************************
函数名:	void duty_200hz(void)
说明: 200hz实时任务函数
***************************************/
void duty_200hz(void)
{
	sdk_duty_run();					  			//SDK总任务控制
	get_wheel_speed();					   		//获取轮胎转速
	motor_output(speed_ctrl_mode); 				//控制器输出
	imu_data_sampling();					 	//加速度计、陀螺仪数据采集
	trackless_ahrs_update();			 		//ahrs姿态更新
}

void duty_1000hz(void)
{

		gpio_input_check_channel_12();//检测12路灰度灰度管状态

}
