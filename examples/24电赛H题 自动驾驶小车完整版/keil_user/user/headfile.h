
#ifndef __HEADFILE_H
#define __HEADFILE_H

#define heater_port PORTB_PORT
#define heater_pin  PORTB_HEATER_PIN

#define KEYUP  GPIOPinRead(keyup_port  ,keyup_pin)
#define KEYDN  GPIOPinRead(keydown_port,keydown_pin)
						 
#include "datatype.h"
#include "main.h"
/*********************************************************/
#include "wp_math.h"
#include "filter.h"
#include "pid.h"
#include "system.h"
#include "ntimer.h"

#include "nqei.h"
#include "nadc.h"
#include "nuart.h"
#include "oled.h"
#include "ssd1306.h"
#include "ni2c.h"
#include "nbutton.h"
#include "ngpio.h"
#include "w25qxx.h"
#include "neeprom.h"
#include "rgb.h"
#include "icm20608.h"
#include "gray_detection.h"
#include "sensor.h"					
#include "motor_control.h"
#include "ui.h"
#include "sdk.h"
#include "bluetooth.h"
#include "subtask.h"
#include "developer_mode.h"
#endif
