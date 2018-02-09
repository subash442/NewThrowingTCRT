/*
 * NewThrowingTCRT.cpp
 *
 * Created: 2/6/2018 9:14:37 AM
 * Author : Deepak Chand
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "Motor.h"
#include "PID.h"
#include "uart.h"
#include "headers.h"
#include "Encoder.h"

#define MAX_SPEED		120
#define RAMP_STEP		2

Motor   M;
Encoder E;
PID    Speed_PID;
PID    Angle_PID;


void Initialize_TCRT_Interrupt(void);
void GoToHome(void);
void Throw(void);

#define TCRT       D,1
#define Pneumatic  L,0

volatile bool PIDFlag   = true;
volatile bool Throwflag = false;
volatile bool Goflag    = false;

unsigned char data;
int TCRTRevolutionCount;
int Speed;
int ocr;


int main(void)
{
	M.Initialise();
	
	E.Encoder_Initialize();
	
	Initialize_TCRT_Interrupt();
	OUTPUT(Pneumatic);
	CLEAR(Pneumatic);
	
	ocr   = 58;
	Speed_PID.Initialize();
	Speed_PID.Set_Range(-249,249);
	Speed_PID.Set_PID(8.05,0.115,4.299);                               //7.64,0.005,2.664   // 10.55,0.059,0.135  15.30,0.074,0.619  14.30,0.149,1.264
	
	Angle_PID.Initialize();
	Angle_PID.Set_Range(-249,249);
	Angle_PID.Set_PID(2.09,0,0.09);
	
	sei();
	uart0_init(UART_BAUD_SELECT(9600,F_CPU));
	uart0_puts("Give Command!!!\n");
	
	//GoToHome();
	int previous_data = 0;
	int8_t dir = -1;         //1 for forward direction and -1 for reverse direction
	int motor_speed = -55;
	Speed = motor_speed;
	
	while (1)
	{
		data = uart0_getc();
		switch (data)
		{
			case 'a':
			{
				Goflag = false;
				PIDFlag = false;
				Angle_PID.SetSetPoint(45);
				E.Angle_count = 0;
				E.ExtraCount = 0;
				break;
			}
			case 'g':
			{
				Goflag = true;
				PIDFlag = true;
				Speed = motor_speed;
				Speed_PID.SetSetPoint(Speed);
				TCRTRevolutionCount = 0;
				E.ExtraCount = 0;
				break;
			}
			case 's':
			{
				Goflag = false;
				PIDFlag = true;
				Speed_PID.SetSetPoint(0);
				M.StopMotor();
				TCRTRevolutionCount = 0;
				break;
			}
			case 'h':
			{
				Goflag = false;
				GoToHome();
				break;
			}
			
			case 'q':
			{
				CLEAR(Pneumatic);
				break;
			}
			case 'w':
			{
				SET(Pneumatic);
				break;
			}
			case 'i':
			{
				motor_speed += 10;
				if (motor_speed >= MAX_SPEED ) motor_speed = MAX_SPEED;
				break;
			}
			case 'd':
			{
				motor_speed -= 10;
				if (motor_speed <= -MAX_SPEED) motor_speed = -MAX_SPEED;
				break;
			}
			default:break;
		}
		data = 0;
		if (Speed_PID.PID_Flag == true && PIDFlag == true)
		{
			M.SetOcrValue(Speed_PID.Compute_PID(E.Encoder_get_speed()));
			Speed_PID.PID_Flag = false;
		}
		if (Angle_PID.PID_Flag == true && PIDFlag == false)
		{
			M.SetOcrValue(Angle_PID.Compute_PID(E.Encoder_Get_angle()));
			Angle_PID.PID_Flag = false;
		}
		if (Throwflag && TCRTRevolutionCount >= 4 )
		{
			if ( abs(E.ExtraCount - previous_data) >= 100 )
			{
				Speed = Speed - (dir*RAMP_STEP);
				previous_data = E.ExtraCount;
			}
			Speed_PID.SetSetPoint(Speed);
			PIDFlag = true;
			
			if ((dir*Speed) <= 0)
			{
				Goflag = false;
				Throwflag = false;
				Speed = 0 ;
			}
		}
		
		
		uart0_putint(motor_speed);
		uart0_putc('\t');
		uart0_putint(TCRTRevolutionCount);
		uart0_putc('\n');
	}
}

ISR(TIMER0_COMPA_vect)
{
	Speed_PID.PID_Flag = true;
	Angle_PID.PID_Flag = true;
	E.Encoder_update_Speed();
}

ISR(ENCODER_INTERRUPT_VECT)
{
	E.Encoder_Increase_Pulse_Counter();
	E.Encoder_Increase_Angle_Counter();
}


void Initialize_TCRT_Interrupt()
{
	INPUT(TCRT);								//Interrupt Pin as Input
	SET(TCRT);									//Pull_UP
	
	EICRA |= (1<<ISC11);						//Falling Edge Interrupt
	EIMSK |= (1<<INT1);
	EIFR  |= (1<<INTF1);
}

void GoToHome()
{
	while(READ(TCRT))
	{
		/*	M1.SetReverseDirection();*/
		M.SetOcrValue(30);
		//E.Angle_count = 0;
			
	}
	M.SetOcrValue(0);
	E.Angle_count = 0;
	TCRTRevolutionCount = 0;
	E.ExtraCount = 0;
	Speed_PID.SetSetPoint(0);
	M.StopMotor();
}

void Throw()
{
		CLEAR(Pneumatic);
		Throwflag = true;
}

ISR(INT1_vect)
{
	E.Angle_count = 0;
	if(Goflag)
		TCRTRevolutionCount++;
	if (TCRTRevolutionCount == 2)
		Throw();
}
