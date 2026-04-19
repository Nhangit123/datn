#include "main.h"
#include "stdio.h"

extern ADC_Obj adc_Objt_var;
extern volatile uint16_t ADC_CH1[NumOfADC_Channel];
extern ADC_HandleTypeDef hadc1;
extern LPF LPF_Vin, LPF_Vout, LPF_I, LPF_Vdclink;
extern LPF LPF_I;

void vAdc_DataInit(ADC_Obj *pObjt)
{
	// Clear ADC raw data buffer receive from DMA
	for (int i = 0; i < NumOfADC_Channel; i++)
	{
		pObjt->ADC_Data[i] = 0;
	}
	// FIX hardware in Pin 2 OPAM has 1.73V
	pObjt->f_I_Sensitivity = 0.035454f; /*V/A*/ // SEN_ACS/0.066 * (3.3/4.3) * 0.7 = 0.035454	// Current sensor sensitivity in V/A
	pObjt->f_ADC_Vref = 3.3f;					// ADC reference voltage
	pObjt->f_V_Divider = 10.0f;					// Voltage divider ratio for Vin/Vout
	pObjt->f_Vdc_Diveder = 55.0f;				// Voltage divider ratio for Vdc
	pObjt->ui32_ADC_Max = AD_MAX_12Bit;			// Max ADC count for 12-bit ADC

	// Compute gains used to convert ADC counts into physical values
	pObjt->f_V_gain = pObjt->f_V_Divider * pObjt->f_ADC_Vref / pObjt->ui32_ADC_Max;
	pObjt->f_Vdc_gain = pObjt->f_Vdc_Diveder * pObjt->f_ADC_Vref / pObjt->ui32_ADC_Max;
	pObjt->f_I_gain = pObjt->f_ADC_Vref / (pObjt->f_I_Sensitivity * pObjt->ui32_ADC_Max);


	// Software calibration factors, currently set to 1.0
	pObjt->f_K_I = 1.0f;
	pObjt->f_K_Vdc = 1.0f;
	pObjt->f_K_Vin = 1.0f;
	pObjt->f_K_Vout = 1.0f;
}
void vADC_Data_Handle(ADC_Obj *pObjt)
{
	uint16_t AD_I_Sen, AD_Vin, AD_Vout, AD_VDC, AD_Vref;

	// Read raw ADC values from DMA buffer
	AD_Vout = ADC_CH1[0];
	AD_Vin = ADC_CH1[1];
	AD_I_Sen = ADC_CH1[2];
	AD_Vref = ADC_CH1[3];
	AD_VDC = ADC_CH1[4];

	// Convert ADC counts to actual voltages
	pObjt->f_Vin_Raw = pObjt->f_V_gain * pObjt->f_K_Vin * AD_Vin;
	pObjt->f_Vout_Raw = pObjt->f_V_gain * pObjt->f_K_Vout * AD_Vout;
	pObjt->f_Vdc_Raw = pObjt->f_Vdc_gain * pObjt->f_K_Vdc * AD_VDC;

	// Apply low-pass filtering to reduce measurement noise
	pObjt->f_Vin_Filter = fLPF_Run(&LPF_Vin, pObjt->f_Vin_Raw);
	pObjt->f_Vout_Filter = fLPF_Run(&LPF_Vout, pObjt->f_Vout_Raw);
	pObjt->f_Vdc_Filter = fLPF_Run(&LPF_Vdclink, pObjt->f_Vdc_Raw);

	// Remove current sensor offset so current becomes a signed value around zero
	pObjt->i_I_Raw = (AD_I_Sen - pObjt->ui16_I_Offset_ADC);
	// Convert current ADC difference into Ampere
	pObjt->f_I_Raw = pObjt->f_I_gain * pObjt->f_K_I * pObjt->i_I_Raw;
	// Filter the current signal
	pObjt->f_I_Filter = fLPF_Run(&LPF_I, pObjt->f_I_Raw);
}
