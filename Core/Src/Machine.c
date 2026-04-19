#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "main.h"

/********************************************************************************/
extern const float f_base_Ts; //PWM isr frequency - 40kHz
extern const float f_base_Tdk;
//Machine
extern machine_control mCtr;
extern machine_parameter mPara;
extern PWM PWM_Var;
//control
extern fuzzySet FzTi, FzAbsEk, FzKp;
extern cPI PIVarV, PIVarIb, PIVarIc;
extern SMC SMC_Ic;
//ADC
extern ADC_Obj adc_Objt_var;
extern volatile uint16_t ADC_CH1[NumOfADC_Channel];
extern ADC_HandleTypeDef hadc1;
//LPF
/* Analog */
extern LPF LPF_Vin, LPF_Vout, LPF_I, LPF_Vdclink;
extern LPF LPF_AD_I_Offset;
/* Controller */
extern LPF LPF_DI;
extern UART_DATA uartdata;
/********************************************************************************/
void MachineParameterConfig(machine_parameter *mPara)
{
	//Circuit
	mPara->L = 0.002f; // H
	mPara->rL = 0.01f; // Ohm
	mPara->C = 5500e-6f; // Fara
}
void StateMachine_Init()
{
	mCtr.enum_mState = mSENSOR_CALIB; // Start with sensor calibration state
}
void StateMachine(void)
{
	static uint16_t uil16_isrcnt_SensCalib = 0; // Counter used during sensor calibration period
    static uint16_t uil16_isrcnt_100us = 0;     // Divider counter to generate a 100 us event
    static uint16_t uil16_100usEvent = 0;       // Flag indicating that the 100 us task should run
	
	//Handle isr counter
	if(++uil16_isrcnt_100us>3) // 100us - used for controller loop
	{
		uil16_100usEvent = 1;
		uil16_isrcnt_100us = 0;
	}
	/*------------------------50us Tasks Fast task section------------------------*/
	if(mCtr.enum_mState == mSENSOR_CALIB)
	{
		//ADC read	
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_CH1, NumOfADC_Channel);

		// Estimate current sensor zero-offset using a low-pass filter
		adc_Objt_var.ui16_I_Offset_ADC = fLPF_Run(&LPF_AD_I_Offset,ADC_CH1[0]);

		// Stay in calibration state for a fixed number of ISR cycles
		if(++uil16_isrcnt_SensCalib > 999)
		{
			uil16_isrcnt_SensCalib = 0;
			mCtr.enum_mState = mRUN;
		}
	}
	else
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_CH1, NumOfADC_Channel);
		vADC_Data_Handle(&adc_Objt_var);
		// Copy ADC measurements into machine feedback variables
		mCtr.f_Ib_fb = adc_Objt_var.f_I_Filter;
		mCtr.f_Vdc_fb = adc_Objt_var.f_Vdc_Filter;
		mCtr.f_Vin = adc_Objt_var.f_Vin_Filter;
		mCtr.f_Vout = adc_Objt_var.f_Vout_Filter;
	}
	if( uil16_100usEvent == 1)
	{

		if(mCtr.enum_mState == mRUN)
		{
			if(mCtr.enum_ctrState == State1)
			{
				ControlLoop_ST1(&mCtr, &PWM_Var);
			}
			else if (mCtr.enum_ctrState == State2)
			{
				ControlLoop_State2(&mCtr, &PWM_Var,&PIVarV, &PIVarIb,&LPF_DI);
			}
			else if (mCtr.enum_ctrState == State2)
			{
			ControlLoop_State3_FullPID(&mCtr, &PWM_Var,&PIVarV, &PIVarIb,&LPF_DI);
			}
			if(mCtr.f_Vdc_fb > ERROR_VOLTAGE)
			{
				mCtr.enum_mState = mFLT;
			}
		}
		else if(mCtr.enum_mState == mFLT)
			{
				PWM_Set_Braking(0.95, 40000);
			}
	}		
}
