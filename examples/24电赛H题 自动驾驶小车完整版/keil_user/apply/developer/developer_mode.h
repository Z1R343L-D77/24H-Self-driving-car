#ifndef __DEVELOPER_MODE_H
#define __DEVELOPER_MODE_H



void sdk_duty_run(void);

extern int16_t sdk_work_mode;

typedef struct 
{
	float ac_x;
		float ac_y;
	float bd_x;
		float bd_y;
	float b_x;
		float b_y;
	float d_x;
		float d_y;
}position;

extern position pos;

#endif


