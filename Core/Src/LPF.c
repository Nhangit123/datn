#include "main.h"


/**********************************************************************************************/
void vLPF_Init(LPF *pLPF, uint16_t ConnerFrqHz, float samplingTime)
{
	/*Low pass filter*/
	float temp1, temp2;
	
	pLPF->Fc = ConnerFrqHz;
	pLPF->Ts = samplingTime;
	pLPF->alpha = 1.0f/(2*PI*pLPF->Ts*pLPF->Fc);
	
	temp1 = (1+pLPF->alpha); 
	temp2 = temp1*temp1;
	pLPF->a1 = (1+2*pLPF->alpha)/temp2; 
	pLPF->a2 = -2*pLPF->alpha/temp2;
	pLPF->b1 = -2*pLPF->alpha/temp1; 
	pLPF->b2 = pLPF->alpha*pLPF->alpha/temp2;
	
	pLPF->yk = 0;
	pLPF->yk_1 = 0; 
	pLPF->yk_2 = 0;
	pLPF->uk = 0;
	pLPF->uk_1 = 0;
}
/**********************************************************************************************/
float fLPF_Run(LPF *pLPF, float input)
{
	//Read LPF input
	pLPF->uk = input;	
	//Compute LPF output
	pLPF->yk = -pLPF->b1*pLPF->yk_1 - pLPF->b2*pLPF->yk_2 + pLPF->a1*pLPF->uk + pLPF->a2*pLPF->uk_1; 
	//Save LPF past data
	pLPF->yk_2 = pLPF->yk_1; 
	pLPF->yk_1 = pLPF->yk; 
	pLPF->uk_1 = pLPF->uk;
	//Return LPF output
	return pLPF->yk;
}
