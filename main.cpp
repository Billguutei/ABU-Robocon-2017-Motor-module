/*
 * Motor Module Test.cpp
 *
 * Created: 4/13/2016 8:48:03 AM
 * Author : Bilguutei
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>


#include "Port_Interrupt.h"
#include "pid.h"
#include "USART.h"


#define		USART_DATA_LENGTH		12
#define		ptr_SPEED				0
#define		ptr_ROTATION			1
#define		ptr_BACK_ANGLE			2
#define		ptr_SIDE_ANGLE			3
#define		ptr_BUTTON				4


#define PI 3.14159265

struct PID_DATA speed_pid;
struct PID_DATA pid_SA;
struct PID_DATA pid_BA;
struct PID_DATA pid_SH;


uint8_t seg_data[12] = { 0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x10, 0x08, 0x06};
uint8_t speed_negj_oron = 0;
uint8_t speed_aravt_oron = 0;
uint8_t table_position = 0;
uint8_t refresh_position = 0;

char tx_buffer[USART_DATA_LENGTH];

int8_t rx_buffer[USART_DATA_LENGTH];
//		<SET (LF Speed 1 byte) (RF Speed 1 byte) (BK Speed 1 byte) (Arm Potition 1 Byte) (Climb 1 byte)>


//int8_t 
char MISSION_REGISTER[7];// = { '0', '0', '0', '0', '0', '0', '0'};
	
//**********************************0********1*******2*******3*******4*******5*******6*******7*******8******9****************	
//uint16_t Shooting_Speed[10] 	= { 0, 		39, 	42, 	54, 	47, 	35, 	42,		45, 	50, 	44,		};
//int16_t	Shooter_BA[10] 		= { 0, 		300, 	300, 	450, 	210, 	210, 	210, 	300, 	470, 	110,	};
//*****************************************************************************************************************	
//**********************************0********1*******2*******3*******4*******5*******6*******7*******8******9****************
//uint16_t Shooting_Speed[12] 	= { 0, 		45, 	42, 	44, 	50, 	35, 	39,		42, 	54, 	47,		0,		0};
//int16_t	Shooter_BA[12] 			= { 0, 		300, 	210, 	110, 	470, 	210, 	300, 	300, 	450, 	210,	0,		0};
//**********************************1********1*******2*******3*******4*******5*******6*******7*******8*******9***************	
//******************************zassan_2in_untsug_golos***********************************************************************************
//uint16_t Shooting_Speed[12] 	= { 0, 		42, 	41, 	44, 	50, 	35, 	39,		42, 	54, 	47,		0,		0};
//int16_t	Shooter_BA[12] 			= { 0, 		500, 	210, 	110, 	470, 	210, 	300, 	300, 	450, 	210,	0,		0};	
//*******************************************1*******2*******3*******4*******5*******6******7********8*******9******10 der hurd_40,untsug_1250********	
//*******************************************************tsenegleh_hesges_shideh**********************************************************	
uint16_t Shooting_Speed[12] 	= { 0, 		39, 	41, 	54, 	47, 	35, 	51,		54, 	50, 	0,		0,		40};
int16_t	Shooter_BA[12] 			= { 0, 		300, 	210, 	450, 	210, 	210, 	450, 	450, 	470, 	0,		0,		1250};


//*****************************************************************************************************************	
//*****************************************************************************************************************	
//*****************************************************************************************************************	
//*****************************************************************************************************************	

unsigned char Header[4]	={'<','S','E','T'};
unsigned char Footer	= '>';
bool mission_incomnig	= false;
bool mission_complete	= false;
bool tx_complete		= true;
uint8_t rx_pointer		= 0;
uint8_t tx_pointer		= 0;

int16_t	LF_SPEED_Mission	= '0';
int16_t	RF_SPEED_Mission	= '0';


uint16_t SH_SPEED_Mission	=  0;
uint16_t SA_SPEED_Mission	=  0;
uint16_t BA_SPEED_Mission	=  0;

//int8_t	ARM_POSITION		= '0';
//int8_t	CLIMB_STATUS		= '0';

//-------- PROCESS VALUES ------------------------------

int32_t	LF_SPEED		= 0;
int32_t	RF_SPEED		= 0;
int16_t	BK_SPEED		= 0;

uint16_t	SH_SPEED		= 0;

uint16_t	SA_SPEED		= 0;
uint16_t	BA_SPEED		= 0;

//uint32_t Robot_Height = 0;

bool  Motor_LF_Speed_Updated = false;
bool  Motor_RF_Speed_Updated = false;

bool  SA_Speed_Updated = false;
bool  BA_Speed_Updated = false;
bool  Motor_SH_Speed_Updated = false;

int32_t	Motor_L_En_0 = 4000;
int32_t	Motor_L_En_1 = 4000;

int32_t	Motor_R_En_0 = 4000;
int32_t	Motor_R_En_1 = 4000;

int32_t Robot_Angle = 0;

uint16_t	Shooter_Motor_En_0 = 0;
uint16_t	Shooter_Motor_En_1 = 0;

int16_t	Motor_BA_En_0 = 10000;
int16_t	Motor_BA_En_1 = 10000;

int16_t	Shooter_BA_Mission = 0;

int16_t	Motor_SA_En_0 = 10000;
int16_t	Motor_SA_En_1 = 10000;

uint8_t Hand_Status = 0;
uint16_t push_time = 0;
uint16_t reload_time = 0;

bool Shoot_Enable = false;
bool Speed_Add_button_Status = false;
bool Speed_Sub_button_Status = false;
bool Select_table_button_Status = false;
bool Select_table_sub_button_Status = false;
bool setting_angle = false;


bool SET_MISSION();
bool BUILD_STATUS();
void TRANSMIT_START();
void PID_CALC();
bool TRIGGER();
bool READY();
bool RELOAD();
bool SHOOT();
void SEVEN_SEG();
void ANGLE_MOTOR_STOP();
void shooter_back_angle_init(void);
void shooter_side_angle_init(void);
void set_angle(void);
//F_CPU=14745600UL
uint8_t temp_UDR0;
bool rx_compelete=false;
uint8_t movement_status = 0;

uint8_t shooter_movement_status = 0;
uint8_t agle_motor_brake_timer = 0;
uint8_t shooter_side_angle_status = 0;

int main(void)
{
	pid_Init(64,32,16,&pid_SA);
	pid_Init(64,32,16,&pid_BA);
	pid_Init(16,8,10,&pid_SH);

	portInit();
	USART_Init(47);
	interruptInit();
	Shooter_Backward;
	shooter_side_angle_init();
	shooter_back_angle_init();
	mission_complete = false;
	while (mission_complete == false)
	{
		Buzzer_Sound();
		_delay_ms(100);
	}
	mission_complete = false;
	Buzzer_Sound();
	Buzzer_Sound();
	Buzzer_Sound();
    while (1)
    {
		if (mission_complete)
		{
			if (MISSION_REGISTER[ptr_SPEED] > 0x85)
			{
				if ((LF_SPEED >= 0) && (RF_SPEED >= 0))
				{
					if (movement_status != 1)
					{
						LEFT_MOTOR_PWM = 0;
						RIGHT_MOTOR_PWM = 0;
						movement_status = 1;
					}
					Left_Motor_Forward;
					Right_Motor_Forward;
					uint8_t speed_multiplier = 8;
					uint16_t pwm_value = (MISSION_REGISTER[ptr_SPEED] - 0x85 ) * speed_multiplier;
					if (MISSION_REGISTER[ptr_SPEED] > 0xC0)
					{
						pwm_value = 0x1FF;
					}
					if (LEFT_MOTOR_PWM < pwm_value)
					{
						if (MISSION_REGISTER[ptr_SPEED] > 0xC0)
						{
							if ((0X1FF - LEFT_MOTOR_PWM) < 15)
							{
								LEFT_MOTOR_PWM = 0X1FF;
								RIGHT_MOTOR_PWM = 0X1FF;
							}
							else
							{
								LEFT_MOTOR_PWM = LEFT_MOTOR_PWM + 15;
								RIGHT_MOTOR_PWM = RIGHT_MOTOR_PWM + 15;
							}
							
						}
						else
						{
							LEFT_MOTOR_PWM = LEFT_MOTOR_PWM + 6;
							RIGHT_MOTOR_PWM = RIGHT_MOTOR_PWM + 6;
						}
					}
					else if (LEFT_MOTOR_PWM > pwm_value)
					{
						LEFT_MOTOR_PWM = pwm_value;
						RIGHT_MOTOR_PWM = pwm_value;
					}
				}
				else
				{
					if (movement_status != 0)
					{
						LEFT_MOTOR_PWM = 0;
						RIGHT_MOTOR_PWM = 0;
						movement_status = 0;
					}
					Left_Motor_Brake;
					Right_Motor_Brake;
					if (LEFT_MOTOR_PWM < 0x1F0)
					{
						LEFT_MOTOR_PWM = LEFT_MOTOR_PWM + 5;
						RIGHT_MOTOR_PWM = LEFT_MOTOR_PWM;
					}
					else
					{
						LEFT_MOTOR_PWM = 0x1FF;
						RIGHT_MOTOR_PWM = 0x1FF;
					}
				}
			}
			else if (MISSION_REGISTER[ptr_SPEED] < 0x75)
			{
				if ((LF_SPEED <= 0) && (RF_SPEED <= 0))
				{
					if (movement_status != 2)
					{
						LEFT_MOTOR_PWM = 0;
						RIGHT_MOTOR_PWM = 0;
						movement_status = 2;
					}
					Left_Motor_Backward;
					Right_Motor_Backward;
					uint8_t speed_multiplier = 7;

					uint16_t pwm_value = (0x7A - MISSION_REGISTER[ptr_SPEED]) * speed_multiplier;
					if (MISSION_REGISTER[ptr_SPEED] < 0x50)
					{
						pwm_value = 0x1FF;
					}
					if (LEFT_MOTOR_PWM < pwm_value)
					{
						if (MISSION_REGISTER[ptr_SPEED] < 0x50)
						{
							if ((0X1FF - LEFT_MOTOR_PWM) < 15)
							{
								LEFT_MOTOR_PWM = 0X1FF;
								RIGHT_MOTOR_PWM = 0X1FF;
							}
							else
							{
								LEFT_MOTOR_PWM = LEFT_MOTOR_PWM + 15;
								RIGHT_MOTOR_PWM = RIGHT_MOTOR_PWM + 15;
							}
						}
						else
						{
							LEFT_MOTOR_PWM = LEFT_MOTOR_PWM + 6;
							RIGHT_MOTOR_PWM = RIGHT_MOTOR_PWM + 6;
						}
					}
					else if (LEFT_MOTOR_PWM > pwm_value)
					{
						LEFT_MOTOR_PWM = pwm_value;
						RIGHT_MOTOR_PWM = pwm_value;
					}
				}
				else
				{
					if (movement_status != 0)
					{
						//if (movement_status == 1)
						//{
							LEFT_MOTOR_PWM = 0x1F0;
							RIGHT_MOTOR_PWM = 0x1F0;
						//}
						//else
						//{
						//	LEFT_MOTOR_PWM = 0;
						//	RIGHT_MOTOR_PWM = 0;
						//}
						movement_status = 0;
					}
					Left_Motor_Brake;
					Right_Motor_Brake;
					if (LEFT_MOTOR_PWM < 0x1F0)
					{
						LEFT_MOTOR_PWM = LEFT_MOTOR_PWM + 5;
						RIGHT_MOTOR_PWM = LEFT_MOTOR_PWM;
					}
					else
					{
						LEFT_MOTOR_PWM = 0x1FF;
						RIGHT_MOTOR_PWM = 0x1FF;
					}
				}
			}
			else
			{
				Left_Motor_Brake;
				Right_Motor_Brake;
				if (movement_status != 0)
				{
					if (movement_status == 1)
					{
						LEFT_MOTOR_PWM = 0x1F0;
						RIGHT_MOTOR_PWM = 0x1F0;
					}
					else
					{
						LEFT_MOTOR_PWM = 0;
						RIGHT_MOTOR_PWM = 0;
					}
					
					movement_status = 0;
				}
				
				if ((LF_SPEED == 0) && (RF_SPEED == 0))
				{
					LEFT_MOTOR_PWM = 0;
					RIGHT_MOTOR_PWM = 0;
				}
				else if (LEFT_MOTOR_PWM < 0x1F0)
				{
					LEFT_MOTOR_PWM = LEFT_MOTOR_PWM + 5;
					RIGHT_MOTOR_PWM = LEFT_MOTOR_PWM;
				}
				else
				{
					LEFT_MOTOR_PWM = 0x1FF;
					RIGHT_MOTOR_PWM = 0x1FF;
				}
			}
//********************************************************
			
//*****************************************************************************************			
			if ((MISSION_REGISTER[ptr_SIDE_ANGLE] > 0x80))
			{
				//if (movement_status == 2) { Front_Motor_Left; }
				//else { 
					Front_Motor_Right;// }
				uint16_t pwm_value = (MISSION_REGISTER[ptr_SIDE_ANGLE] - 0x80 ) * 6;
				if ((FRONT_MOTOR_PWM < pwm_value))// && (LEFT_MOTOR_PWM < 0x1FF))
				{
					if (MISSION_REGISTER[ptr_SIDE_ANGLE] > 0xB0)
					{
						
						if ((0x1FF - FRONT_MOTOR_PWM) < 15) { FRONT_MOTOR_PWM = 0X1FF; }
						else { FRONT_MOTOR_PWM = FRONT_MOTOR_PWM + 15; }
						if (movement_status == 0)
						{
							Right_Motor_Forward;
							RIGHT_MOTOR_PWM = FRONT_MOTOR_PWM / 2;
							Left_Motor_Backward;
							LEFT_MOTOR_PWM = FRONT_MOTOR_PWM / 2;
							movement_status = 3;
						}
							
												
					}
					else { FRONT_MOTOR_PWM = FRONT_MOTOR_PWM + 6; }
				}
				else if (FRONT_MOTOR_PWM > pwm_value) { FRONT_MOTOR_PWM = pwm_value; }
			}
			else if (MISSION_REGISTER[ptr_SIDE_ANGLE] < 0x80)
			{
				//if (movement_status == 2) { Front_Motor_Right; }
				//else { 
					Front_Motor_Left;// }

				uint16_t pwm_value = (0x80 -MISSION_REGISTER[ptr_SIDE_ANGLE]) * 6;
				if ((FRONT_MOTOR_PWM < pwm_value))
				{
					if (MISSION_REGISTER[ptr_SIDE_ANGLE] < 0x40)
					{
						if ((0X1FF - FRONT_MOTOR_PWM) < 15) { FRONT_MOTOR_PWM = 0X1FF; }
						else { FRONT_MOTOR_PWM = FRONT_MOTOR_PWM + 15; }
						if (movement_status == 0)
						{	
							Right_Motor_Backward;
							RIGHT_MOTOR_PWM = FRONT_MOTOR_PWM / 2;
							Left_Motor_Forward;
							LEFT_MOTOR_PWM = FRONT_MOTOR_PWM / 2;
							movement_status = 3;
						}
					}
					else { FRONT_MOTOR_PWM = FRONT_MOTOR_PWM + 3; }
				}
				else if (FRONT_MOTOR_PWM > pwm_value) { FRONT_MOTOR_PWM = pwm_value; }
			}
			else { Front_Motor_Brake; FRONT_MOTOR_PWM = 0; }
			
//*********************************************************************************************	
			// Ariin untsug deeshllune
			//if (MISSION_REGISTER[ptr_BACK_ANGLE] > 0xB0)
			//{
				//if (shooter_movement_status != 1)
				//{
					//ANGLE_MOTOR_STOP();
					//shooter_movement_status = 1;
					//BACK_MOTOR_ON;
					//_delay_ms(50);
					//Angle_Motor_Up;
				//}
				//if (!Back_angle_Down_Pushed)
				//{
					//BA_SPEED_Mission = 5;
					//if(BA_Speed_Updated)
					//{
						//ANGLE_MOTOR_PWM = pid_Controller( BA_SPEED_Mission, BA_SPEED, &pid_BA);
						//BA_Speed_Updated = false;
					//}
				//}
				//else
				//{
					//ANGLE_MOTOR_STOP();
					//ANGLE_MOTOR_PWM = 0x1FF;
					//shooter_movement_status = 0;
				//}
			//}
			//// Ariin untsug dooshllulna
			//else if (MISSION_REGISTER[ptr_BACK_ANGLE] < 0x40)
			//{
				//
				//if (shooter_movement_status != 2)
				//{
					//ANGLE_MOTOR_STOP();
					//shooter_movement_status = 2;
					//BACK_MOTOR_ON;
					//_delay_ms(50);
					//Angle_Motor_Down;
				//}
				//if (!Back_angle_Top_Pushed)
				//{
					//BA_SPEED_Mission = 3;
					//if(BA_Speed_Updated)
					//{
						//ANGLE_MOTOR_PWM = pid_Controller( BA_SPEED_Mission, BA_SPEED, &pid_BA);
						//BA_Speed_Updated = false;
					//}
				//}
				//else
				//{
					//ANGLE_MOTOR_STOP();
					//ANGLE_MOTOR_PWM = 0x1FF;
					//shooter_movement_status = 0;
				//}
			//}
			//else if (MISSION_REGISTER[ptr_SIDE_ANGLE] > 0xB0)
			//{
				//if (shooter_movement_status != 3)
				//{
					//ANGLE_MOTOR_STOP();
					//shooter_movement_status = 3;
					//SIDE_MOTOR_ON;
					//_delay_ms(50);
					//Angle_Motor_Down;
				//}
				//if (!Side_angle_Top_Pushed)
				//{
					//SA_SPEED_Mission = 3;
					//if(SA_Speed_Updated)
					//{
						//ANGLE_MOTOR_PWM = pid_Controller( SA_SPEED_Mission, SA_SPEED, &pid_SA);
						//SA_Speed_Updated = false;
					//}
				//}
				//else
				//{
					//ANGLE_MOTOR_STOP();
					//ANGLE_MOTOR_PWM = 0x1FF;
					//shooter_movement_status = 0;
				//}
			//}
			//else if (MISSION_REGISTER[ptr_SIDE_ANGLE] < 0x40)
			//{
				//if (shooter_movement_status != 4)
				//{
					//ANGLE_MOTOR_STOP();
					//shooter_movement_status = 4;
					//SIDE_MOTOR_ON;
					//_delay_ms(50);
					//Angle_Motor_Up;
				//}
				//if (!Side_angle_Down_Pushed)
				//{
					//SA_SPEED_Mission = 5;
					//if(SA_Speed_Updated)
					//{
						//ANGLE_MOTOR_PWM = pid_Controller( SA_SPEED_Mission, SA_SPEED, &pid_SA);
						//SA_Speed_Updated = false;
					//}
				//}
				//else
				//{
					//ANGLE_MOTOR_STOP();
					//ANGLE_MOTOR_PWM = 0x1FF;
					//shooter_movement_status = 0;
				//}
			//}
			//else if (setting_angle == false)
			//{
				//if (shooter_movement_status != 0)
				//{
					//ANGLE_MOTOR_STOP();
					//BACK_MOTOR_ON;
				//}
				//ANGLE_MOTOR_PWM = 0x1FF;
				//shooter_movement_status = 0;
			//}
			
			
//**********  SHOOT ENABLE *********************************************************************************************

			if (((MISSION_REGISTER[ptr_BUTTON] & 0x04) == 0x04))
			{
				if (SH_SPEED == SH_SPEED_Mission) { Shoot_Enable = true; }
			}
			
//**********  SHOOTER SPEED ADD *********************************************************************************************

			if ((MISSION_REGISTER[ptr_BUTTON] & 0x09) == 0x09)
			{
				if (Speed_Add_button_Status == false)
				{
					if (SH_SPEED_Mission == 0) { SH_SPEED_Mission = 50; }
					else if (SH_SPEED_Mission < 90) { SH_SPEED_Mission ++; }
					Speed_Add_button_Status = true;
				}
				buzzer_on;
			}
			else 
			{ 
				if (Speed_Add_button_Status)
				{
					buzzer_off;
				}
				Speed_Add_button_Status = false; 
			}
				
//**********  SHOOTER SPEED SUB  *****************************************************************************

			if ((MISSION_REGISTER[ptr_BUTTON] & 0x0A) == 0x0A)
			{
				if (Speed_Sub_button_Status == false)
				{
					if (SH_SPEED_Mission > 0) { SH_SPEED_Mission --; }
					else { SH_SPEED_Mission = 0; SHOOTER_MOTOR_PWM = 0; }
					Speed_Sub_button_Status = true;
				}
				buzzer_on;
			}
			else 
			{ 
				if (Speed_Sub_button_Status)
				{
					buzzer_off;
				}
				Speed_Sub_button_Status = false; 
			}
				
//**********   TABLE POSITION CHANGER  *****************************************************************************************	
			if ((MISSION_REGISTER[ptr_BUTTON] & 0x09) == 0x01)
			{
				if (Select_table_button_Status == false)
				{
					if (table_position < 11) { table_position ++; }
					else { table_position = 0; }
					Select_table_button_Status = true;
					
				}
				buzzer_on;
			}
			else 
			{ 
				if (Select_table_button_Status)
				{
					buzzer_off;
				}
				Select_table_button_Status = false; 
			}
				
			if ((MISSION_REGISTER[ptr_BUTTON] & 0x0A) == 0x02)
			{
				if (Select_table_sub_button_Status == false)
				{
					if (table_position > 0) { table_position --; }
					else { table_position = 0; }
					Select_table_sub_button_Status = true;
					
				}
				
				buzzer_on;
			}
			else 
			{ 
				if (Select_table_sub_button_Status)
				{
					buzzer_off;
				}
				Select_table_sub_button_Status = false; 
			}
			
			//int16_t temp_sh_speed = Motor_BA_En_1;
			
			//int16_t temp_sh_speed = SH_SPEED;
			//itoa(temp_sh_speed, tx_buffer, 10);
			//BUILD_STATUS();
			mission_complete = false;
			
		}
		if ((Select_table_button_Status) || (Select_table_sub_button_Status))
		{
			Shooter_BA_Mission = Shooter_BA[table_position];
			SH_SPEED_Mission = Shooting_Speed[table_position];
			setting_angle = true;
		}
		
		
		if ((Shooter_BA_Mission + 5) > Motor_BA_En_1)
		{
			setting_angle = true;
		}
		if (setting_angle)
		{
			set_angle();
		}
		SHOOT();	
		SEVEN_SEG();	
		_delay_ms(5);

    }
}
bool SHOOT()
{
	if (Shoot_Enable)
	{
		if (Hand_Status == 1)
		{
			PUSH_MOTOR_PWM = 0;
			Push_Motor_Forward;
			if ((Hand_Front_Pushed) || (push_time > 20000))
			{
				Push_Motor_Brake;
				PUSH_MOTOR_PWM = 0x1FF;
				Hand_Status = 2;
				reload_time = 0;
				push_time = 0;
			}
			else
			{
				PUSH_MOTOR_PWM = 0x1FF;
				push_time ++;
			}
		}
		else
		{
			PUSH_MOTOR_PWM = 0;
			Push_Motor_Backward;
			if ((Hand_Back_Pushed) || (reload_time > 20000))
			{
				Push_Motor_Brake;
				PUSH_MOTOR_PWM = 0x1FF;
				Shoot_Enable = false;
				Hand_Status = 1;
				reload_time = 0;
				push_time = 0;
			}
			else
			{
				PUSH_MOTOR_PWM = 0x1FF;
				reload_time ++;
			}
		}
	}
	else
	{
		Push_Motor_Brake;
		PUSH_MOTOR_PWM = 0x1FF;
	}
	PID_CALC();
	return true;
}

void SEVEN_SEG()
{
	speed_aravt_oron = SH_SPEED / 10;
	speed_negj_oron = SH_SPEED % 10;
	if (refresh_position == 0)
	{
		SEG_1_OFF; SEG_2_OFF; SEG_3_OFF; SEG_4_OFF;
		PORTA = seg_data[table_position];
		SEG_1_ON;
		refresh_position = 1;
	}
	else if (refresh_position == 1)
	{
		SEG_1_OFF; SEG_2_OFF; SEG_3_OFF; SEG_4_OFF;
		PORTA = 0x3F;
		SEG_2_ON;
		refresh_position = 2;
	}
	else if (refresh_position == 2)
	{
		SEG_1_OFF; SEG_2_OFF; SEG_3_OFF; SEG_4_OFF;
		PORTA = seg_data[speed_aravt_oron];
		SEG_3_ON;
		refresh_position = 3;
	}
	else
	{
		SEG_1_OFF; SEG_2_OFF; SEG_3_OFF; SEG_4_OFF;
		PORTA = seg_data[speed_negj_oron];
		SEG_4_ON;
		refresh_position = 0;
	}
	//switch refresh_position
}
void ANGLE_MOTOR_STOP()
{
	Angle_Motor_Brake;
	ANGLE_MOTOR_PWM = 0;
	BA_SPEED = 0;
	SA_SPEED = 0;
	pid_Reset_Integrator(&pid_BA);
	pid_Reset_Integrator(&pid_SA);
}
void shooter_side_angle_init(void)
{
	if (Side_angle_Top_Pushed == false)
	{
		ANGLE_MOTOR_STOP();
		SIDE_MOTOR_ON;
		_delay_ms(50);
		//ANGLE_MOTOR_PWM = 0;
		Angle_Motor_Down;
		while (Side_angle_Top_Pushed == false)
		{
			SA_SPEED_Mission = 5;
			if(SA_Speed_Updated)
			{
				ANGLE_MOTOR_PWM = pid_Controller( SA_SPEED_Mission, SA_SPEED, &pid_SA);
				SA_Speed_Updated = false;
			}
			_delay_ms(10);
		}
		shooter_side_angle_status = 0;
		ANGLE_MOTOR_STOP();
	}
	Motor_SA_En_1 = 0;
	Motor_SA_En_0 = 0;
}
void shooter_side_angle_up_init(void)
{
	if (Side_angle_Down_Pushed == false)
	{
		ANGLE_MOTOR_STOP();
		SIDE_MOTOR_ON;
		_delay_ms(50);
		//ANGLE_MOTOR_PWM = 0;
		Angle_Motor_Up;
		while (Side_angle_Down_Pushed == false)
		{
			SA_SPEED_Mission = 10;
			if(SA_Speed_Updated)
			{
				ANGLE_MOTOR_PWM = pid_Controller( SA_SPEED_Mission, SA_SPEED, &pid_SA);
				SA_Speed_Updated = false;
			}
			_delay_ms(10);
		}
		shooter_side_angle_status = 1;
		ANGLE_MOTOR_STOP();
	}
	
	//Motor_SA_En_1 = 0;
	//Motor_SA_En_0 = 0;
	
}
void shooter_back_angle_init(void)
{	
	ANGLE_MOTOR_STOP();
	BACK_MOTOR_ON;
	_delay_ms(50);
	//ANGLE_MOTOR_PWM = 0;
	Angle_Motor_Down;
	while (Back_angle_Top_Pushed == false)
	{
		BA_SPEED_Mission = 5;
		if(BA_Speed_Updated)
		{
			ANGLE_MOTOR_PWM = pid_Controller( BA_SPEED_Mission, BA_SPEED, &pid_BA);
			BA_Speed_Updated = false;
		}
		_delay_ms(10);
	}
	Motor_BA_En_1 = 0;
	Motor_BA_En_0 = 0;
	ANGLE_MOTOR_STOP();
}

void set_angle(void)
{
	if ((table_position == 11) && ((shooter_side_angle_status != 1) && (600 < Motor_BA_En_1)))
	{
		LEFT_MOTOR_PWM = 0;
		RIGHT_MOTOR_PWM = 0;
		FRONT_MOTOR_PWM = 0;
		PUSH_MOTOR_PWM = 0;
		shooter_side_angle_up_init();
		shooter_movement_status = 0;
	}
	else if ((table_position != 11) && ((shooter_side_angle_status != 0) && (700 > Motor_BA_En_1)))
	{
		LEFT_MOTOR_PWM = 0;
		RIGHT_MOTOR_PWM = 0;
		FRONT_MOTOR_PWM = 0;
		PUSH_MOTOR_PWM = 0;
		shooter_side_angle_init();
		shooter_movement_status = 0;
	}
	if ((Shooter_BA_Mission + 5) < Motor_BA_En_1)
	{
		if (shooter_movement_status != 2)
		{
			ANGLE_MOTOR_STOP();
			shooter_movement_status = 2;
			BACK_MOTOR_ON;
			//_delay_ms(50);
			Angle_Motor_Down;
		}
		if (!Back_angle_Top_Pushed)
		{
			BA_SPEED_Mission = 5;
			if(BA_Speed_Updated)
			{
				ANGLE_MOTOR_PWM = pid_Controller( BA_SPEED_Mission, BA_SPEED, &pid_BA);
				BA_Speed_Updated = false;
			}
		}
		else
		{
			if (shooter_movement_status != 0)
			{
				ANGLE_MOTOR_STOP();
				BACK_MOTOR_ON;
			}
			ANGLE_MOTOR_PWM = 0x1FF;
			shooter_movement_status = 0;
			Motor_BA_En_1 = 0;
			Motor_BA_En_0 = 0;
			setting_angle = false;
		}
		
		
	}
	else if ((Shooter_BA_Mission + 5) > Motor_BA_En_1)
	{
		if (shooter_movement_status != 1)
		{
			ANGLE_MOTOR_STOP();
			shooter_movement_status = 1;
			BACK_MOTOR_ON;
			//_delay_ms(50);
			Angle_Motor_Up;
		}
		if (!Back_angle_Down_Pushed)
		{
			BA_SPEED_Mission = 8;
			if(BA_Speed_Updated)
			{
				ANGLE_MOTOR_PWM = pid_Controller( BA_SPEED_Mission, BA_SPEED, &pid_BA);
				BA_Speed_Updated = false;
			}
		}
		else
		{
			if (shooter_movement_status != 0)
			{
				ANGLE_MOTOR_STOP();
				BACK_MOTOR_ON;
			}
			ANGLE_MOTOR_PWM = 0x1FF;
			shooter_movement_status = 0;
			setting_angle = false;
		}
	}
	else
	{
		if (shooter_movement_status != 0)
		{
			ANGLE_MOTOR_STOP();
			BACK_MOTOR_ON;
		}
		ANGLE_MOTOR_PWM = 0x1FF;
		shooter_movement_status = 0;
		setting_angle = false;
	}
}


ISR(USART0_RX_vect)
{
	uint16_t temp_SREG = SREG;
	char _UDR0 = UDR0;
	if ((_UDR0 == 0x0A) && (mission_incomnig == false))
	{
		if (mission_complete == false)
		{
			mission_incomnig = true;
			rx_pointer = 0;
		}	
	} 
	else if ((_UDR0 == 0x0A) && (mission_incomnig == true))
	{
		if (rx_pointer == 5)
		{
			mission_complete = true;
			mission_incomnig = false;
		}
		else if (rx_pointer == 0)
		{
			//mission_incomnig = false;
		}
		else if (rx_pointer < 5)
		{
			MISSION_REGISTER[rx_pointer] = _UDR0;
			rx_pointer ++;
		}
	}
	else if (mission_incomnig)
	{
		MISSION_REGISTER[rx_pointer] = _UDR0;
		rx_pointer ++;
	}
	UDR0 = _UDR0;
	SREG = temp_SREG;
}

ISR(USART0_TX_vect)
{
	uint16_t temp_SREG = SREG;
	//if (tx_pointer < (USART_DATA_LENGTH - 1))
	//{
		//tx_pointer ++;
		//UDR0 = tx_buffer[tx_pointer];
	//}
	//else
	//{
		//tx_complete = true;
		//mission_complete = false;
		//mission_incomnig = false;
	//}
	SREG = temp_SREG;
}

uint8_t shooter_timer = 0;
ISR(TIMER2_OVF_vect)//, ISR_NOBLOCK)
{
	uint16_t temp_SREG = SREG;
	
//----------- Motor Front Left Speed Calculation ---------------------
	Robot_Angle = Motor_L_En_1 - Motor_R_En_1;
	//if ( Motor_L_En_1 >= Motor_L_En_0 )
	//{
		LF_SPEED		= Motor_L_En_1 - Motor_L_En_0;
		Motor_LF_Speed_Updated = true;
	//}
		Motor_L_En_0	= Motor_L_En_1;
	
//----------- Motor Front Right Speed Calculation ---------------------	

	//if ( Motor_RF_En_1 >= Motor_RF_En_0 )
	//{
		RF_SPEED		= Motor_R_En_1 - Motor_R_En_0;
		Motor_RF_Speed_Updated = true;
	//}
		Motor_R_En_0	= Motor_R_En_1;
	
//----------- Motor Side Speed Calculation ---------------------
		
	if (Motor_SA_En_1 > Motor_SA_En_0)
	{
		SA_SPEED = Motor_SA_En_1 - Motor_SA_En_0;
	} 
	else
	{
		SA_SPEED = Motor_SA_En_0 - Motor_SA_En_1;
	}
	Motor_SA_En_0 = Motor_SA_En_1; SA_Speed_Updated = true;
	
//----------- Motor Back Speed Calculation ---------------------
	
	if (Motor_BA_En_1 > Motor_BA_En_0)
	{
		BA_SPEED = Motor_BA_En_1 - Motor_BA_En_0;
	}
	else
	{
		BA_SPEED = Motor_BA_En_0 - Motor_BA_En_1;
	}
	Motor_BA_En_0 = Motor_BA_En_1; BA_Speed_Updated = true;
//----------- Shooter Speed Calculation ---------------------
	if (shooter_timer >= 2)
	{
		if ( Shooter_Motor_En_1 >= Shooter_Motor_En_0 )
		{
			SH_SPEED		= Shooter_Motor_En_1 - Shooter_Motor_En_0;
			Motor_SH_Speed_Updated = true;
		}
		Shooter_Motor_En_0	= Shooter_Motor_En_1;
		shooter_timer = 0;
	} 
	else
	{
		shooter_timer ++;
	}
	

	SREG = temp_SREG;
}


/* MOTOR RIGHT ENCODER ROUTE En-6, BIDIRECTIONAL*/
ISR(INT0_vect)
{
	uint16_t temp_SREG = SREG;
	if ((PIND & 0x10) == 0x00)
	{
		Motor_R_En_1 ++;
	}
	else
	{
		if (Motor_R_En_1 > 0)
		{
			Motor_R_En_1 --;
			//Buzzer_Sound();
		}
	}
	SREG = temp_SREG;
}
/*SHOOTER MOTOR ENCODER ROUTE En-1*/
ISR(INT1_vect)
{
	uint16_t temp_SREG = SREG;
	Shooter_Motor_En_1 ++;
	SREG = temp_SREG;
}
/* MOTOR LEFT ENCODER ROUTE En-2, BIDIRECTIONAL*/
ISR(INT7_vect)
{
	uint16_t temp_SREG = SREG;
	if ((PIND & 0x40) == 0x40)
	{
		Motor_L_En_1 ++;
	}
	else
	{
		if (Motor_L_En_1 > 0)
		{
			Motor_L_En_1 --;
			//Buzzer_Sound();
		}
	}
	SREG = temp_SREG;
}
/* MOTOR BACK ANGLE ENCODER ROUTE En-4, BIDIRECTIONAL*/
ISR(INT6_vect)
{
	uint16_t temp_SREG = SREG;
	if ((PIND & 0x20) != 0x20)
	{
		Motor_BA_En_1 ++;
	}
	else
	{
		if (Motor_BA_En_1 > 0)
		{
			Motor_BA_En_1 --;
			//Buzzer_Sound();
		}
	}
	SREG = temp_SREG;
}
/* MOTOR SIDE ANGLE ENCODER ROUTE En-3, BIDIRECTIONAL*/
ISR(INT2_vect) // pole
{
	uint16_t temp_SREG = SREG;
	if ((PIND & 0x80) != 0x80)
	{
		Motor_SA_En_1 ++;
	}
	else
	{
		if (Motor_SA_En_1 > 0)
		{
			Motor_SA_En_1 --;
			//Buzzer_Sound();
		}
	}
	SREG = temp_SREG;
}

bool BUILD_STATUS()
{
	if (tx_complete)
	{
		//for ( uint8_t i = 0; i < 7; i ++)
		//{
			//tx_buffer[i + 4] = MISSION_REGISTER[i];		
		//}
		//
		tx_complete = false;
		TRANSMIT_START();
		
		
		return true;
	}
	return false;
}
void TRANSMIT_START()
{
	tx_pointer = 0;
	UDR0 = tx_buffer[tx_pointer];
}
void PID_CALC()
{
	
	//if(BA_Speed_Updated)	{	ANGLE_MOTOR_PWM = pid_Controller( RF_SPEED_Mission, RF_SPEED, &pid_BA);	BA_Speed_Updated = false;}
	//if(SA_Speed_Updated)	{	ANGLE_MOTOR_PWM = pid_Controller( LF_SPEED_Mission, LF_SPEED, &pid_SA);	SA_Speed_Updated = false;}
	if(Motor_SH_Speed_Updated)	{	SHOOTER_MOTOR_PWM = pid_Controller( SH_SPEED_Mission, SH_SPEED, &pid_SH);	Motor_SH_Speed_Updated = false;}
	
}
