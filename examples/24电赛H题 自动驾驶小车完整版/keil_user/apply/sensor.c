#include "headfile.h"
#include "sensor.h"
#include "Fusion.h"

lpf_param accel_lpf_param,gyro_lpf_param;
lpf_buf gyro_filter_buf[3],accel_filter_buf[3];

sensor smartcar_imu;
FusionAhrs ahrs;
FusionOffset offset;
#define sampling_frequent 200

/***************************************************
函数名: vector3f_sub(vector3f a,vector3f b,vector3f *c)
说明:	向量作差
入口:	vector3f a-被减数
			vector3f b-减数
			vector3f *c-差
****************************************************/
void vector3f_sub(vector3f a,vector3f b,vector3f *c)
{
	c->x=a.x-b.x;
  c->y=a.y-b.y;
  c->z=a.z-b.z;
}

/***************************************************
函数名: void euler_to_quaternion(float *rpy,float *q)
说明:	欧拉角转四元数
入口:	float *rpy-欧拉角
			float *q-四元数
****************************************************/
void euler_to_quaternion(float *rpy,float *q)
{
	float sPitch2, cPitch2; // sin(phi/2) and cos(phi/2)
	float sRoll2 , cRoll2;  // sin(theta/2) and cos(theta/2)
	float sYaw2  , cYaw2;   // sin(psi/2) and cos(psi/2)
	//calculate sines and cosines
	
	FastSinCos(0.5f * rpy[0]*DEG2RAD, &sRoll2, &cRoll2);//roll
	FastSinCos(0.5f * rpy[1]*DEG2RAD, &sPitch2,&cPitch2);//pitch
	FastSinCos(0.5f * rpy[2]*DEG2RAD, &sYaw2,  &cYaw2);//yaw
	
	// compute the quaternion elements
	q[0] = cPitch2*cRoll2*cYaw2+sPitch2*sRoll2*sYaw2;
	q[1] = sPitch2*cRoll2*cYaw2-cPitch2*sRoll2*sYaw2;
	q[2] = cPitch2*sRoll2*cYaw2+sPitch2*cRoll2*sYaw2;
	q[3] = cPitch2*cRoll2*sYaw2-sPitch2*sRoll2*cYaw2;

  // Normalise quaternion
  float recipNorm = invSqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}

/***************************************************
函数名: void calculate_quaternion_init(vector3f a,vector3f m,float *q)
说明: 通过加速度计、磁力计数据计算初始四元数
入口:	vector3f a-三轴加速度计
			vector3f m-三轴磁力计
			float *q-四元数
****************************************************/
void calculate_quaternion_init(vector3f a,vector3f m,float *q)
{
	float ax,ay,az,mx,my,mz;
	float rpy_obs_deg[3],_sin_rpy[2],_cos_rpy[2];
	
	ax= a.x;	ay= a.y;	az= a.z;
	mx= m.x;	my= m.y;	mz= m.z;	
	float norm = invSqrt(mx * mx + my * my + az * az);
	mx *=	norm;
	my *=	norm;
	mz *=	norm;
	
	  rpy_obs_deg[0]=-57.3f*atan(ax*invSqrt(ay*ay+az*az)); //横滚角
	  rpy_obs_deg[1]= 57.3f*atan(ay*invSqrt(ax*ax+az*az)); //俯仰角
	_sin_rpy[_PIT] =	sinf(rpy_obs_deg[1]*DEG2RAD);
	_cos_rpy[_PIT] =	cosf(rpy_obs_deg[1]*DEG2RAD);
	_sin_rpy[_ROL] =	sinf(rpy_obs_deg[0]*DEG2RAD);
	_cos_rpy[_ROL] =	cosf(rpy_obs_deg[0]*DEG2RAD);

  /************磁力计倾角补偿*****************/
	vector2f magn;	
	magn.y=  my * _cos_rpy[_PIT]- mz * _sin_rpy[_PIT];
	magn.x=  mx * _cos_rpy[_ROL]
					+my * _sin_rpy[_ROL] * _cos_rpy[_PIT]
					+mz * _sin_rpy[_ROL] * _cos_rpy[_PIT];
  /***********反正切得到磁力计观测角度*********/
  rpy_obs_deg[2] = atan2f(magn.x,magn.y)*57.296f;
	if(rpy_obs_deg[2]<0) rpy_obs_deg[2] = rpy_obs_deg[2]+360;
	rpy_obs_deg[2] = constrain_float(rpy_obs_deg[2],0.0f,360);
	//
	euler_to_quaternion(rpy_obs_deg,q);//计算观测四元数
}
/***************************************************
函数名: void imu_data_sampling(void)
说明:	IMU数据采样/校准/滤波
****************************************************/
void imu_data_sampling(void)
{
	if(smartcar_imu.lpf_init==0)
	{
		set_cutoff_frequency(200, 50,&gyro_lpf_param); //姿态角速度反馈滤波参数 
		set_cutoff_frequency(200, 30,&accel_lpf_param);//姿态解算加计修正滤波值
	  	smartcar_imu.lpf_init=1;
	}
	smartcar_imu.last_temperature_raw=smartcar_imu.temperature_raw;
	//陀螺仪/陀螺仪数据采集
	ICM206xx_Read_Data(&smartcar_imu._gyro_dps_raw,&smartcar_imu._accel_g_raw,&smartcar_imu.temperature_raw);
	//陀螺仪数据低通滤波
	smartcar_imu.gyro_dps_raw.x=LPButterworth(smartcar_imu._gyro_dps_raw.x,&gyro_filter_buf[0],&gyro_lpf_param);
	smartcar_imu.gyro_dps_raw.y=LPButterworth(smartcar_imu._gyro_dps_raw.y,&gyro_filter_buf[1],&gyro_lpf_param);
	smartcar_imu.gyro_dps_raw.z=LPButterworth(smartcar_imu._gyro_dps_raw.z,&gyro_filter_buf[2],&gyro_lpf_param);		
	//加速度数据低通滤波
	smartcar_imu.accel_g_raw.x=LPButterworth(smartcar_imu._accel_g_raw.x,&accel_filter_buf[0],&accel_lpf_param);
	smartcar_imu.accel_g_raw.y=LPButterworth(smartcar_imu._accel_g_raw.y,&accel_filter_buf[1],&accel_lpf_param);
	smartcar_imu.accel_g_raw.z=LPButterworth(smartcar_imu._accel_g_raw.z,&accel_filter_buf[2],&accel_lpf_param);	
	//温度传感器数据一阶低通滤波
	smartcar_imu.temperature_filter=0.75f*smartcar_imu.temperature_raw+0.25f*smartcar_imu.temperature_filter;
 	//得到校准后的角速度、加速度
	vector3f_sub(smartcar_imu.gyro_dps_raw,smartcar_imu.gyro_offset,&smartcar_imu.gyro_dps);
  
	smartcar_imu.accel_g.x=smartcar_imu.accel_scale.x*smartcar_imu.accel_g_raw.x-smartcar_imu.accel_offset.x;
  	smartcar_imu.accel_g.y=smartcar_imu.accel_scale.y*smartcar_imu.accel_g_raw.y-smartcar_imu.accel_offset.y;
  	smartcar_imu.accel_g.z=smartcar_imu.accel_scale.z*smartcar_imu.accel_g_raw.z-smartcar_imu.accel_offset.z;  	
	//加速度计/陀螺仪校准检测
	imu_calibration(&smartcar_imu.gyro_dps_raw,&smartcar_imu.accel_g_raw);
}

/***************************************************
函数名: void trackless_ahrs_update(void)
说明:	姿态更新
****************************************************/
void trackless_ahrs_update(void)
{
	FusionVector gyroscope={0.0f, 0.0f, 0.0f};
	FusionVector accelerometer = {0.0f, 0.0f, 1.0f};
	FusionVector earthacceleration = {0.0f, 0.0f, 0.0f}; 
	
	gyroscope.axis.x=smartcar_imu.gyro_dps.x;
	gyroscope.axis.y=smartcar_imu.gyro_dps.y;
	gyroscope.axis.z=smartcar_imu.gyro_dps.z;
	
	accelerometer.axis.x=smartcar_imu.accel_g.x;
	accelerometer.axis.y=smartcar_imu.accel_g.y;
	accelerometer.axis.z=smartcar_imu.accel_g.z;

	if(smartcar_imu.quaternion_init_ok==0)
	{
			calculate_quaternion_init(smartcar_imu.accel_g,smartcar_imu.mag_tesla,smartcar_imu.quaternion_init);
			smartcar_imu.quaternion_init_ok	=	1;
			//AHRS初始化
			FusionOffsetInitialise(&offset, sampling_frequent);
			FusionAhrsInitialise(&ahrs);
			//Set AHRS algorithm settings
			const FusionAhrsSettings settings = {
							.gain = 0.5f,
							.accelerationRejection = 10.0f,
							.magneticRejection = 20.0f,
							.rejectionTimeout = 5 * sampling_frequent, /* 5 seconds */
			};
			FusionAhrsSetSettings(&ahrs, &settings);		
	}

	if(smartcar_imu.quaternion_init_ok==1)
	{
		gyroscope = FusionOffsetUpdate(&offset, gyroscope);
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer,0.005f);//FusionAhrsUpdateExternalHeading(&ahrs, gyroscope, accelerometer,uav_ahrs.rpy_obs_deg[2],0.005f);
		FusionEuler euler=FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		earthacceleration=FusionAhrsGetEarthAcceleration(&ahrs);
		smartcar_imu.rpy_deg[_ROL]=euler.angle.pitch;
		smartcar_imu.rpy_deg[_PIT]=euler.angle.roll;
		smartcar_imu.rpy_deg[_YAW]=euler.angle.yaw;
	}
	smartcar_imu.rpy_gyro_dps[_PIT]=smartcar_imu.gyro_dps.x;
	smartcar_imu.rpy_gyro_dps[_ROL]=smartcar_imu.gyro_dps.y;
	smartcar_imu.rpy_gyro_dps[_YAW]=smartcar_imu.gyro_dps.z;
	//计算姿态相关三角函数
	smartcar_imu.sin_rpy[_PIT]=FastSin(smartcar_imu.rpy_deg[_PIT]*DEG2RAD);
	smartcar_imu.cos_rpy[_PIT]=FastCos(smartcar_imu.rpy_deg[_PIT]*DEG2RAD);
	smartcar_imu.sin_rpy[_ROL]=FastSin(smartcar_imu.rpy_deg[_ROL]*DEG2RAD);
	smartcar_imu.cos_rpy[_ROL]=FastCos(smartcar_imu.rpy_deg[_ROL]*DEG2RAD);
	smartcar_imu.sin_rpy[_YAW]=FastSin(smartcar_imu.rpy_deg[_YAW]*DEG2RAD);
	smartcar_imu.cos_rpy[_YAW]=FastCos(smartcar_imu.rpy_deg[_YAW]*DEG2RAD);
	
	//{-sinθ          cosθsin Φ                          cosθcosΦ                   }
	//计算航向加速度
	smartcar_imu.yaw_gyro_enu=-smartcar_imu.sin_rpy[_ROL]*smartcar_imu.gyro_dps.x
							+smartcar_imu.cos_rpy[_ROL]*smartcar_imu.sin_rpy[_PIT]*smartcar_imu.gyro_dps.y
							+smartcar_imu.cos_rpy[_PIT]*smartcar_imu.cos_rpy[_ROL]*smartcar_imu.gyro_dps.z;	
														
}
/***************************************************
函数名: void get_wheel_speed(void)
说明:	车载里程计系统
备注:	在姿态解算就位后才开始计算
****************************************************/
void get_wheel_speed(void)
{
	static uint32_t cnt=0;
	if(cnt<200)  { cnt++; return; }
	smartcar_imu.left_motor_speed_cmps =get_left_motor_speed();
	smartcar_imu.right_motor_speed_cmps=get_right_motor_speed();
	//计算两轮平均速度
	smartcar_imu.state_estimation.speed=0.5f*(smartcar_imu.left_motor_speed_cmps+smartcar_imu.right_motor_speed_cmps);
	smartcar_imu.state_estimation.azimuth=smartcar_imu.rpy_deg[_YAW];//航向角
	smartcar_imu.state_estimation.w=smartcar_imu.yaw_gyro_enu;//航向加速度
	//计算x,y方向上的速度分量
	smartcar_imu.state_estimation.vel.x=smartcar_imu.state_estimation.speed*cosf(DEG2RAD*smartcar_imu.state_estimation.azimuth);
	smartcar_imu.state_estimation.vel.y=smartcar_imu.state_estimation.speed*sinf(DEG2RAD*smartcar_imu.state_estimation.azimuth);
	//速度直接积分得到两个方向的位置
	smartcar_imu.state_estimation.pos.x+=smartcar_imu.state_estimation.vel.x*0.005f;
	smartcar_imu.state_estimation.pos.y+=smartcar_imu.state_estimation.vel.y*0.005f;
	//平均速度直接积分总距离
	smartcar_imu.state_estimation.distance+=smartcar_imu.state_estimation.speed*0.005f;
}


