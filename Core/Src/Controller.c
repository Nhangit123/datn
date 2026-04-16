#include "main.h"
#include "math.h"
#include "stdio.h"

extern fuzzySet FzTi, FzAbsEk, FzKp;
extern cPI PIVarV, PIVarIb, PIVarIc;
extern SMC SMC_Ic;
extern LPF LPF_DI;
extern machine_control mCtr;
extern UART_DATA uartdata;
/**************************************PI************************************************/
void vPI_Init(cPI *pPI, float Ts, float Kp, float TI, float lamda, float Usat)
{
	// Reset internal PI states
	pPI->Uk_1 = 0;
	pPI->Uk = 0;
	pPI->Usk_1 = 0;
	pPI->Ek = 0;
	pPI->Ek_1 = 0;
	pPI->Esk = 0;
	pPI->Esk_1 = 0;

	// Output saturation limits
	pPI->Umax = Usat;
	pPI->Umin = -Usat;

	// PI parameters
	pPI->Ts = Ts;	   // Sampling period
	pPI->Kp = Kp;	   // Proportional gain
	pPI->Ks = pPI->Kp; // Gain used in anti-windup correction
	pPI->Ti = TI;	   // Integral time constant
	pPI->D = 1.0f - pPI->Ts / pPI->Ti;
}
void vPI_Refresh(cPI *pPI)
{
	pPI->Ek_1 = 0.0f;
	pPI->Uk_1 = 0.0f;
	pPI->Usk_1 = 0.0f;
	pPI->Ek = 0.0f;
	pPI->Uk = 0.0f;
	pPI->Usk = 0.0f;
}
float fPI_Run(cPI *pPI, float Ref, float Fb, uint8_t ui8_Mode)
{
	// Compute current control error
	pPI->Ek = Ref - Fb;

	// Select controller mode: P or PI
	// Note: this code switches on pPI->ui8_Mode, not on the function argument ui8_Mode
	switch (pPI->ui8_Mode)
	{
	case P:
	{
		// Pure proportional control
		pPI->Uk = pPI->Kp * pPI->Ek;

		// Apply output saturation
		if (pPI->Uk > pPI->Umax)
			pPI->Usk = pPI->Umax;
		else if (pPI->Uk < pPI->Umin)
			pPI->Usk = pPI->Umin;
		else
			pPI->Usk = pPI->Uk;
		break;
	}

	case P_I:
	{
		// Discrete PI controller with anti-windup
		pPI->Esk_1 = pPI->Ek_1 + (1 / pPI->Ks) * (pPI->Usk_1 - pPI->Uk_1);
		pPI->Uk = pPI->Usk_1 + pPI->Kp * (pPI->Ek - pPI->D * pPI->Esk_1);

		// Apply output saturation
		if (pPI->Uk > pPI->Umax)
			pPI->Usk = pPI->Umax;
		else if (pPI->Uk < pPI->Umin)
			pPI->Usk = pPI->Umin;
		else
			pPI->Usk = pPI->Uk;

		// Update memory for next sampling cycle
		pPI->Ek_1 = pPI->Ek;
		pPI->Uk_1 = pPI->Uk;
		pPI->Usk_1 = pPI->Usk;
		break;
	}
	}

	return pPI->Usk; // Return saturated control output
}
void fPI_UpdateParam(cPI *pPI, float Kp, float Ti)
{
	pPI->Kp = Kp;
	pPI->Ti = Ti;
	pPI->D = 1.0f - pPI->Ts / Ti;
}

/************************************** FUZZY ************************************************/
// Initialize fuzzy membership sets and output levels
// FzAbsEk : fuzzy sets for absolute error magnitude
// FzTi    : fuzzy output set for integral time Ti
// FzKp    : fuzzy output set for proportional gain Kp
void fuzzyInit(void)
{
	// Parameters of input MFs
	FzAbsEk.PZ.fCenter = 0;
	FzAbsEk.PZ.fWidth = 5;
	FzAbsEk.PS.fCenter = 3;
	FzAbsEk.PS.fWidth = 5;
	FzAbsEk.PM.fCenter = 6;
	FzAbsEk.PM.fWidth = 5;
	FzAbsEk.PL.fCenter = 9;
	FzAbsEk.PL.fWidth = 5;
	FzAbsEk.PH.fCenter = 12;
	FzAbsEk.PH.fWidth = 5;
	FzAbsEk.iNumOfMFs = 5;
	for (int i = 0; i < FzAbsEk.iNumOfMFs; i++)
		FzAbsEk.fGrade[i] = 0;

	// Parameters of output MFs Ti
	FzTi.PZ.fCenter = 3e-3;
	FzTi.PZ.fWidth = 2e-3; // 0.0025;0.075    FzTi.PZ.fCenter = 0.014; FzTi.PZ.fWidth = 0.0665*2
	FzTi.PS.fCenter = 4e-3;
	FzTi.PS.fWidth = 2e-3; // 0.01
	FzTi.PM.fCenter = 5e-3;
	FzTi.PM.fWidth = 2e-3; // 0.0175
	FzTi.PL.fCenter = 6e-3;
	FzTi.PL.fWidth = 2e-3; // 0.025
	FzTi.PH.fCenter = 4e-3;
	FzTi.PH.fWidth = 2e-3; // 0.125
	FzTi.iNumOfMFs = 5;
	for (int i = 0; i < FzTi.iNumOfMFs; i++)
		FzTi.fGrade[i] = 0;

	FzKp.PZ.fCenter = 0.003;
	FzKp.PZ.fWidth = 0.02; // 0.015 0.03
	FzKp.PS.fCenter = 0.02;
	FzKp.PS.fWidth = 0.02; // 0.03
	FzKp.PM.fCenter = 0.03;
	FzKp.PM.fWidth = 0.02; // 0.045
	FzKp.PL.fCenter = 0.04;
	FzKp.PL.fWidth = 0.02; // 0.06
	FzKp.PH.fCenter = 0.05;
	FzKp.PH.fWidth = 0.02; // 0.075    0.1 0.04
	FzKp.iNumOfMFs = 5;
	for (int i = 0; i < FzKp.iNumOfMFs; i++)
		FzKp.fGrade[i] = 0;
}

// Convert the numeric input into fuzzy membership grades
// Example: determine how much the current error belongs to
// PZ, PS, PM, PL, and PH sets
void fuzzifier(float input)
{
	// Compute the grade of PZ corresponding to input
	if (input >= 0 && input <= FzAbsEk.PZ.fCenter + FzAbsEk.PZ.fWidth / 2)
		FzAbsEk.fGrade[0] = (2 * (FzAbsEk.PZ.fCenter - input) + FzAbsEk.PZ.fWidth) / FzAbsEk.PZ.fWidth;
	else
		FzAbsEk.fGrade[0] = 0;
	// Compute the grade of PS corresponding to input
	if (input >= FzAbsEk.PS.fCenter - FzAbsEk.PS.fWidth / 2 && input <= FzAbsEk.PS.fCenter)
		FzAbsEk.fGrade[1] = (2 * (input - FzAbsEk.PS.fCenter) + FzAbsEk.PS.fWidth) / FzAbsEk.PS.fWidth;
	else if (input > FzAbsEk.PS.fCenter && input <= FzAbsEk.PS.fCenter + FzAbsEk.PS.fWidth / 2)
		FzAbsEk.fGrade[1] = (2 * (FzAbsEk.PS.fCenter - input) + FzAbsEk.PS.fWidth) / FzAbsEk.PS.fWidth;
	else
		FzAbsEk.fGrade[1] = 0;
	// Compute the grade of PM corresponding to input
	if (input >= FzAbsEk.PM.fCenter - FzAbsEk.PM.fWidth / 2 && input <= FzAbsEk.PM.fCenter)
		FzAbsEk.fGrade[2] = (2 * (input - FzAbsEk.PM.fCenter) + FzAbsEk.PM.fWidth) / FzAbsEk.PM.fWidth;
	else if (input > FzAbsEk.PM.fCenter && input <= FzAbsEk.PM.fCenter + FzAbsEk.PM.fWidth / 2)
		FzAbsEk.fGrade[2] = (2 * (FzAbsEk.PM.fCenter - input) + FzAbsEk.PM.fWidth) / FzAbsEk.PM.fWidth;
	else
		FzAbsEk.fGrade[2] = 0;
	// Compute the grade of PL corresponding to input
	if (input >= FzAbsEk.PL.fCenter - FzAbsEk.PL.fWidth / 2 && input <= FzAbsEk.PL.fCenter)
		FzAbsEk.fGrade[3] = (2 * (input - FzAbsEk.PL.fCenter) + FzAbsEk.PL.fWidth) / FzAbsEk.PL.fWidth;
	else if (input > FzAbsEk.PL.fCenter && input <= FzAbsEk.PL.fCenter + FzAbsEk.PL.fWidth / 2)
		FzAbsEk.fGrade[3] = (2 * (FzAbsEk.PL.fCenter - input) + FzAbsEk.PL.fWidth) / FzAbsEk.PL.fWidth;
	else
		FzAbsEk.fGrade[3] = 0;
	// Compute the grade of PH corresponding to input
	if (input >= FzAbsEk.PH.fCenter - FzAbsEk.PH.fWidth / 2 && input <= FzAbsEk.PH.fCenter)
		FzAbsEk.fGrade[4] = (2 * (input - FzAbsEk.PH.fCenter) + FzAbsEk.PH.fWidth) / FzAbsEk.PH.fWidth;
	else if (input > FzAbsEk.PH.fCenter)
		FzAbsEk.fGrade[4] = 1;
	else
		FzAbsEk.fGrade[4] = 0;
}

// Convert fuzzy membership grades back into crisp output values
// These functions produce actual Ti and Kp values using weighted averaging
float defuzzifier_Ti(float *Ti)
{
	double fNum, fDen;
	// Compute Ti
	fNum = FzAbsEk.fGrade[0] * FzTi.PH.fCenter * FzTi.PH.fWidth + FzAbsEk.fGrade[1] * FzTi.PL.fCenter * FzTi.PL.fWidth + FzAbsEk.fGrade[2] * FzTi.PM.fCenter * FzTi.PM.fWidth + FzAbsEk.fGrade[3] * FzTi.PS.fCenter * FzTi.PS.fWidth + FzAbsEk.fGrade[4] * FzTi.PZ.fCenter * FzTi.PZ.fWidth;
	fDen = FzAbsEk.fGrade[0] * FzTi.PH.fWidth + FzAbsEk.fGrade[1] * FzTi.PL.fWidth + FzAbsEk.fGrade[2] * FzTi.PM.fWidth + FzAbsEk.fGrade[3] * FzTi.PS.fWidth + FzAbsEk.fGrade[4] * FzTi.PZ.fWidth;
	return *Ti = fNum / fDen;
}
// Convert fuzzy membership grades back into crisp output values
// These functions produce actual Ti and Kp values using weighted averaging
float defuzzifier_Kp(float *Kp)
{
	double fNum1 = FzAbsEk.fGrade[0] * FzKp.PZ.fCenter * FzKp.PZ.fWidth + FzAbsEk.fGrade[1] * FzKp.PS.fCenter * FzKp.PS.fWidth + FzAbsEk.fGrade[2] * FzKp.PM.fCenter * FzKp.PM.fWidth + FzAbsEk.fGrade[3] * FzKp.PL.fCenter * FzKp.PL.fWidth + FzAbsEk.fGrade[4] * FzKp.PH.fCenter * FzKp.PH.fWidth;
	double fDen1 = FzAbsEk.fGrade[0] * FzKp.PZ.fWidth + FzAbsEk.fGrade[1] * FzKp.PS.fWidth + FzAbsEk.fGrade[2] * FzKp.PM.fWidth + FzAbsEk.fGrade[3] * FzKp.PL.fWidth + FzAbsEk.fGrade[4] * FzKp.PH.fWidth;
	return *Kp = fNum1 / fDen1;
}
/************************************** SMC ************************************************/
void SMC_Init(SMC *s, float lamda, float k)
{
	s->lamda = lamda;
	s->k = k;
	s->erIk = s->erIk_1 = 0;
	s->s = 0;
}

float SMC_Run(SMC *s, float ref, float fb)
{
	s->erIk = ref - fb;
	s->s = s->erIk + s->lamda * (s->erIk - s->erIk_1);
	s->erIk_1 = s->erIk;
	return s->s;
}
float sign(float s)
{
	if (s > 0)
		return 1;
	if (s < 0)
		return -1;
	return 0;
}
/************************************Control Loop ******************************************/
/***************************** Control Loop: Generic Fuzzy-PI ******************************/
float ControlLoop_FuzzyPI(cPI *pPI, float Ref, float Fb, float ref_in_fz, float fb_ip_fz)
{
	/* 1. fuzzy input */
	fuzzifier(fabs(ref_in_fz - fb_ip_fz));

	/* 2. update PI parameter online */
	fPI_UpdateParam(pPI, defuzzifier_Kp(&pPI->Kp), defuzzifier_Ti(&pPI->Ti));

	/* 3. PI controller */
	fPI_Run(pPI, Ref, Fb, P_I);

	return fPI_Run(pPI, Ref, Fb, P_I);
}
/***************************** Control Loop: SMC ******************************/
float ControlLoop_SMC(SMC *smc, float Ref, float Fb, float Vdc, float L, float rL, float Tdk)
{
	float out;

	/* 1. error */
	smc->erIk = Ref - Fb;

	/* 2. sliding surface */
	smc->s = smc->erIk + smc->lamda * (smc->erIk - smc->erIk_1);

	/* 3. control law */
	out = -L / (Tdk * Vdc) * (Ref - (1 - rL * Tdk / L) * Fb - Tdk / L * Vdc - smc->lamda * smc->erIk - smc->k * sign(smc->s));

	/* 4. update memory */
	smc->erIk_1 = smc->erIk;

	return out;
}
/***************************** Control State ******************************/
// In Here only cal Ic then using UART send to SC board
// Control loop state 1
void ControlLoop_ST1(machine_control *mctr, PWM *PWM_var)
{
	PWM_var->dutyhigh = 0;
	PWM_var->dutylow = 0;

	// Once DC-link voltage is high enough, move to the next state
	if (mctr->f_Vdc_fb > LMTST0)
	{
		mctr->enum_ctrState = State2;
	}
	// PWM
	PWM_Set(PWM_var->dutylow, 40000);
}
// Control loop state 2
void ControlLoop_State2(machine_control *mctr, PWM *PWM_var, cPI *pPIV, cPI *pPIIB, LPF *LPFDI)
{
	// OUTTER LOOP voltage control
	// PIControl voltage
	mctr->f_It_sp = fPI_Run(pPIV, mctr->f_Vdc_SP * mctr->f_Vdc_SP, mctr->f_Vdc_fb * mctr->f_Vdc_fb, pPIV->ui8_Mode);

	// Filter the total current reference to reduce sudden changes
	// before sending it to the inner current loop.
	mctr->f_Ift_sp = fLPF_Run(LPFDI, mctr->f_It_sp);

	// Limit the battery current reference to the allowed maximum range.
	// If the requested current is too high, clamp it to MAX_CURRENT_OF_BAT.
	// If it is too low, clamp it to -MAX_CURRENT_OF_BAT.
	if (mctr->f_Ib_sp > MAX_CURRENT_OF_BAT)
	{
		mctr->f_Ib_sp = MAX_CURRENT_OF_BAT;
	}
	else if (mctr->f_Ib_sp < -MAX_CURRENT_OF_BAT)
	{
		mctr->f_Ib_sp = -MAX_CURRENT_OF_BAT;
	}
	else
	{
		// Otherwise, use the filtered current reference directly.
		mctr->f_Ib_sp = mctr->f_Ift_sp;
	}

	// Inner battery current loop:
	// Convert battery current reference into PWM duty command.
	PWM_var->dutylow = fPI_Run(pPIIB, mctr->f_Ib_sp, mctr->f_Ib_fb, pPIIB->ui8_Mode);

	// Clamp duty cycle to a safe operating range.
	// This avoids too small or too large duty values.
	if (PWM_var->dutylow > MAX_DUTY)
	{
		PWM_var->dutylow = MAX_DUTY;
	}
	else if (PWM_var->dutylow < MIN_DUTY)
	{
		PWM_var->dutylow = MIN_DUTY;
	}
	else
	{
		PWM_var->dutylow = PWM_var->dutylow;
	}

	// Move to State3 when the DC-link voltage reaches the next threshold
	if (mctr->f_Vdc_fb > LMTST1)
	{
		mctr->enum_ctrState = State3;
		vPI_Refresh(pPIV);
		pPIV->ui8_Mode = P_I;
	}
	// PWM
	PWM_Set(PWM_var->dutylow, 40000);
}
// control state 3 used Full PID
void ControlLoop_State3_FullPID(machine_control *mctr, PWM *PWM_var, cPI *pPIV, cPI *pPIIB, LPF *LPFDI)
{
	// Toggle BYPASS pin for debugging or state indication.
	HAL_GPIO_TogglePin(BYPASS_GPIO_Port, BYPASS_Pin);
	// OUTTER LOOP voltage control
	// PIControl voltage
	mctr->f_It_sp = fPI_Run(pPIV, mctr->f_Vdc_SP * mctr->f_Vdc_SP, mctr->f_Vdc_fb * mctr->f_Vdc_fb, pPIV->ui8_Mode);
	// LPF DIVI High and Low fre pass of current
	mctr->f_Ift_sp = fLPF_Run(LPFDI, mctr->f_It_sp);
	// Send output signal current controller
	if (mctr->f_Ib_sp > MAX_CURRENT_OF_BAT)
	{
		mctr->f_Ib_sp = MAX_CURRENT_OF_BAT;
	}
	else if (mctr->f_Ib_sp < -MAX_CURRENT_OF_BAT)
	{
		mctr->f_Ib_sp = -MAX_CURRENT_OF_BAT;
	}
	else
	{
		mctr->f_Ib_sp = mctr->f_Ift_sp;
	}

	// ICsp
	mctr->f_Ic_sp = mctr->f_It_sp - mctr->f_Ib_sp;

	// calculate duty off battery
	PWM_var->dutylow = fPI_Run(pPIIB, mctr->f_Ib_sp, mctr->f_Ib_fb, pPIIB->ui8_Mode);
	if (PWM_var->dutylow > MAX_DUTY)
	{
		PWM_var->dutylow = MAX_DUTY;
	}
	else if (PWM_var->dutylow < MIN_DUTY)
	{
		PWM_var->dutylow = MIN_DUTY;
	}
	else
	{
		PWM_var->dutylow = PWM_var->dutylow;
	}
	// PWM
	PWM_Set(PWM_var->dutylow, 40000);
}
// control state 3 used Fz
void ControlLoop_State3_FZ(machine_control *mctr, PWM *PWM_var, cPI *pPIV, cPI *pPIIB, LPF *LPFDI)
{
	HAL_GPIO_TogglePin(BYPASS_GPIO_Port, BYPASS_Pin);

	// Fuzzify the absolute DC-link voltage error.
	// This error is used to tune PI parameters online.
	fuzzifier(fabs(mctr->f_Vdc_SP - mctr->f_Vdc_fb));

	// Update outer-loop PI parameters using fuzzy logic.
	// Kp and Ti are recalculated based on the current voltage error.
	fPI_UpdateParam(pPIV, defuzzifier_Kp(&pPIV->Kp), defuzzifier_Ti(&pPIV->Ti));

	// Outer voltage loop:
	// Generate the total current reference from the DC-link voltage error.
	// Squared voltage values are used in this control law.
	mctr->f_It_sp = fPI_Run(pPIV, mctr->f_Vdc_SP * mctr->f_Vdc_SP, mctr->f_Vdc_fb * mctr->f_Vdc_fb, pPIV->ui8_Mode);

	// Filter the total current reference to smooth the command.
	mctr->f_Ift_sp = fLPF_Run(LPFDI, mctr->f_It_sp);

	// Limit the battery current reference to the allowed range.
	if (mctr->f_Ib_sp > MAX_CURRENT_OF_BAT)
	{
		mctr->f_Ib_sp = MAX_CURRENT_OF_BAT;
	}
	else if (mctr->f_Ib_sp < -MAX_CURRENT_OF_BAT)
	{
		mctr->f_Ib_sp = -MAX_CURRENT_OF_BAT;
	}
	else
	{
		// Use the filtered current reference directly if it is within limits.
		mctr->f_Ib_sp = mctr->f_Ift_sp;
	}

	// The capacitor current reference is the remaining part of total current
	// after assigning the battery current.
	mctr->f_Ic_sp = mctr->f_It_sp - mctr->f_Ib_sp;

	// Inner battery current loop:
	// Compute PWM duty from the battery current tracking error.
	PWM_var->dutylow = fPI_Run(pPIIB, mctr->f_Ib_sp, mctr->f_Ib_fb, pPIIB->ui8_Mode);

	// Clamp PWM duty to safe bounds.
	if (PWM_var->dutylow > MAX_DUTY)
	{
		PWM_var->dutylow = MAX_DUTY;
	}
	else if (PWM_var->dutylow < MIN_DUTY)
	{
		PWM_var->dutylow = MIN_DUTY;
	}
	else
	{
		PWM_var->dutylow = PWM_var->dutylow;
	}
	
	// PWM
	PWM_Set(PWM_var->dutylow, 40000);
}

/*------------------------------------- Cal SOC--------------------------*/
float SOC(float input, float V_max)
{
	return input * 100.0f / V_max;
}
