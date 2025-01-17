#include "headfile.h"
#include "sdk.h"
#include "subtask.h"
#include "user.h"


controller_output trackless_output={
	.unlock_flag=LOCK,
};

nav_ctrl ngs_nav_ctrl={
	.ctrl_finish_flag=1,
	.update_flag=0,
	.cnt=0,
	.dis_cm=0,
}; 
//航向角度、角速度控制器
float	steer_angle_expect=0,steer_angle_error=0,steer_angle_output=0;
float	steer_gyro_expect=0,steer_gyro_output=0;
float	steer_gyro_scale=steer_gyro_scale_default;//0.5f

controller steergyro_ctrl;		//转向角速度内环控制器结构体//角速度控制参数   1.2  0  3							 
controller distance_ctrl,azimuth_ctrl;//两轮差速位置、航向控制

void e2prom_init(void)
{
	//预留参数
	float tmp_reserved_params[20];
	for(uint16_t i=0;i<8;i++)
	{
		ReadFlashParameterOne(RESERVED_PARAMS_1+i,&tmp_reserved_params[i]);
	}
	if(isnan(tmp_reserved_params[0])==0) 	pos.b_x=tmp_reserved_params[0];	else pos.b_x=200;
	if(isnan(tmp_reserved_params[1])==0) 	pos.b_y=tmp_reserved_params[1];	else pos.b_y=-2;
	if(isnan(tmp_reserved_params[2])==0) 	pos.d_x=tmp_reserved_params[2];	else pos.d_x=-40;
	if(isnan(tmp_reserved_params[3])==0) 	pos.d_y=tmp_reserved_params[3];	else pos.d_y=-85;
	if(isnan(tmp_reserved_params[4])==0) 	pos.ac_x=tmp_reserved_params[4];else pos.ac_x=207;
	if(isnan(tmp_reserved_params[5])==0) 	pos.ac_y=tmp_reserved_params[5];else pos.ac_y=-142;
	if(isnan(tmp_reserved_params[6])==0) 	pos.bd_x=tmp_reserved_params[6];else pos.bd_x=-50;
	if(isnan(tmp_reserved_params[7])==0) 	pos.bd_y=tmp_reserved_params[7];else pos.bd_y=-136;
}
/***************************************************
函数名: void ctrl_params_init(void)
说明:	pid控制器参数初始化
****************************************************/
void ctrl_params_init(void)
{
	pid_control_init(&seektrack_ctrl[0],//待初始化控制器结构体
										turn_kp_default1,//比例参数
										turn_ki_default1,//积分参数
										turn_kd_default1,//微分参数
										20, //偏差限幅值
										0,  //积分限幅值
										500,//控制器输出限幅值
										1,	//偏差限幅标志位
										0,0,//积分分离标志位与引入积分控制时的限幅值
										6); //微分间隔时间


	pid_control_init(&azimuth_ctrl,//待初始化控制器结构体
										5.0f,//比例参数
										0,	 //积分参数
										0,	 //微分参数
										50,  //偏差限幅值
										0,   //积分限幅值
										500, //控制器输出限幅值
										1,   //偏差限幅标志位
										0,0, //积分分离标志位与引入积分控制时的限幅值
										1);  //微分间隔时间

	pid_control_init(&steergyro_ctrl,//待初始化控制器结构体
										steer_gyro_kp_default,//比例参数
										steer_gyro_ki_default,//积分参数
										steer_gyro_kd_default,//微分参数
										200, //偏差限幅值
										300, //积分限幅值
										500, //控制器输出限幅值
										1,  	//偏差限幅标志位
										0,0, //积分分离标志位与引入积分控制时的限幅值
										1);  //微分间隔时间
	
	pid_control_init(&distance_ctrl,//待初始化控制器结构体
										0.5,//比例参数
										0,//积分参数
										0,//微分参数
										75, //偏差限幅值
										0, //积分限幅值
										500, //控制器输出限幅值
										1,  	//偏差限幅标志位
										0,0, //积分分离标志位与引入积分控制时的限幅值
										1);  //微分间隔时间
}

vector2f target={0,0};
/***************************************************
函数名: void get_distance_error(vector2f t)
说明:	获取目标位置和当前位置的偏差
入口:	vector2f t-目标位置
****************************************************/
float get_distance_error(vector2f t)
{
	return sqrtf(sq(t.x-smartcar_imu.state_estimation.pos.x)+sq(t.y-smartcar_imu.state_estimation.pos.y));
}
/***************************************************
函数名: void position_control(void)
说明:	二维目标位置控制
备注:	
		y+(初始车头左侧)
		*
		*
		*******x+(初始车头方向)
****************************************************/
void position_control(void)//已任意姿态到达目标点,无速度末端角度约束
{
	if(ngs_nav_ctrl.update_flag==1)
	{
		target.x=ngs_nav_ctrl.x;
		target.y=ngs_nav_ctrl.y;
		ngs_nav_ctrl.update_flag=0;
		ngs_nav_ctrl.ctrl_finish_flag=0;
		ngs_nav_ctrl.cnt=0;	
	}
	//航向控制
	float theta=atan2f((target.y-smartcar_imu.state_estimation.pos.y),(target.x-smartcar_imu.state_estimation.pos.x));//目标航向
	//角度控制
	azimuth_ctrl.expect=theta*RAD2DEG;
	azimuth_ctrl.feedback=smartcar_imu.rpy_deg[_YAW];
	azimuth_ctrl.error=azimuth_ctrl.expect-azimuth_ctrl.feedback;
	/***********************偏航角偏差超过+-180处理*****************************/
	if(azimuth_ctrl.error<-180) azimuth_ctrl.error=azimuth_ctrl.error+360;
	if(azimuth_ctrl.error> 180) azimuth_ctrl.error=azimuth_ctrl.error-360;				
	azimuth_ctrl.error=constrain_float(azimuth_ctrl.error,-30,30);
	azimuth_ctrl.output=azimuth_ctrl.kp*azimuth_ctrl.error;
	
	//限制最大期望角速度，轮子与底边摩擦力小时改下此值，能减小里程计的距离误差
	steer_angle_output=constrain_float(azimuth_ctrl.output,-120,120);
	
	steergyro_ctrl.expect=steer_angle_output;//期望
	steergyro_ctrl.feedback=smartcar_imu.yaw_gyro_enu;//偏航角度反馈
	pid_control_run(&steergyro_ctrl);		  //控制器运算
	steer_gyro_output=steergyro_ctrl.output;

	//距离控制
	distance_ctrl.expect=0;
	distance_ctrl.feedback=-get_distance_error(target);
	distance_ctrl.error=distance_ctrl.expect-distance_ctrl.feedback;
	distance_ctrl.error=constrain_float(distance_ctrl.error,-180,180);
	distance_ctrl.output=distance_ctrl.kp*distance_ctrl.error;
	distance_ctrl.output=constrain_float(distance_ctrl.output,-90,90);//对速度期望进行限制，避免轮胎打滑造成里程计误差
	
	//航点遍历结束标志判断
	if(ngs_nav_ctrl.cnt<10)//持续50ms满足
	{
		if(ABS(distance_ctrl.error)<=3.0f) ngs_nav_ctrl.cnt++;
		else ngs_nav_ctrl.cnt/=2;
	}
	else
	{
		ngs_nav_ctrl.ctrl_finish_flag=1;
		ngs_nav_ctrl.cnt=0;
	}
	
	if(ngs_nav_ctrl.ctrl_finish_flag==1)//到达目标位置后，速度期望给0
	{
		steer_gyro_output=0;
		distance_ctrl.output=0;
	}
}
/***************************************************
函数名: void distance_control(void)
说明:	距离控制
****************************************************/
void distance_control(void)
{
	//距离控制
	distance_ctrl.feedback=smartcar_imu.state_estimation.distance;
	distance_ctrl.error=distance_ctrl.expect-distance_ctrl.feedback;
	distance_ctrl.error=constrain_float(distance_ctrl.error,-50,50);
	distance_ctrl.output=3.0f*distance_ctrl.error;//
	distance_ctrl.output=constrain_float(distance_ctrl.output,-50,50);//对速度期望进行限制，避免轮胎打滑造成里程计误差
}


