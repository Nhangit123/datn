/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/*******************************************************/
// COntroll parametter
//PI controller
typedef struct
{
    float Kp, Ti;                 // PI controller gains: proportional gain and integral time
    float Ks, D;                  // Internal coefficients used in anti-windup / discrete PI update
    float Umax, Umin;             // Output saturation limits
    float Usk, Usk_1;             // Saturated control output at current and previous sample
    float Uk, Uk_1;               // Unsaturated control output at current and previous sample
    float Ek, Ek_1;               // Control error at current and previous sample
    float Esk, Esk_1;             // Internal anti-windup error terms
    float Ts;                     // Sampling period
    uint8_t ui8_Mode;             // Controller mode: P or PI
} cPI;

//FUZZY controller
typedef struct
{
    float fCenter, fWidth;        // Center and width of a symmetric triangular membership function
} symTriMFs;

typedef struct
{
    symTriMFs PZ, PS, PM, PL, PH; // Linguistic fuzzy sets: Zero, Small, Medium, Large, High
    float fGrade[5];              // Membership grades corresponding to the 5 fuzzy sets
    int iNumOfMFs;                // Number of membership functions used in this fuzzy set
} fuzzySet;

//SMC controller
typedef struct{
    float lamda, k;
    float erIk, erIk_1;
    float s;
}SMC;
/*******************************************************/
typedef struct
{
    float Ts;                     // Sampling time
    float Fc;                     // Corner frequency of the low-pass filter
    float a1, a2, b1, b2;         // Discrete filter coefficients
    float alpha;                  // Additional tuning parameter
    float yk, yk_1, yk_2;         // Current and previous filter outputs
    float uk, uk_1;               // Current and previous filter inputs
} LPF;
/*******************************************************/
typedef struct
{
 float a0, a1, a2, b0, b1, b2;
 float f0, fs, omega;
 float r;
 float G; 
 float yk, yk_1, yk_2, uk, uk_1, uk_2;
}NotchFilter;
/*******************************************************/
// Analog input -12 bit resolution

typedef struct {
    uint16_t ADC_Data[5];         // ADC DMA raw buffer for all channels

    float f_ADC_Vref;             // ADC reference voltage
    uint32_t ui32_ADC_Max;        // Maximum ADC count based on ADC resolution

    float f_V_Divider;            // Divider ratio for Vin/Vout channels
    float f_Vdc_Diveder;          // Divider ratio for Vdc channel
    float f_I_Sensitivity;        // Current sensor sensitivity in V/A
    uint16_t ui16_I_Offset_ADC;   // ADC offset value corresponding to 0A current

    float f_V_gain;               // Gain to convert ADC count into Vin/Vout voltage
    float f_I_gain;               // Gain to convert ADC count into current
    float f_Vdc_gain;             // Gain to convert ADC count into Vdc voltage

    float f_K_I, f_K_Vin, f_K_Vout, f_K_Vdc; // Software calibration factors

    float f_I_Raw, f_Vin_Raw, f_Vout_Raw, f_Vdc_Raw; // Raw physical values before filtering
    int i_I_Raw;                  // Signed current ADC value after subtracting offset
    float f_I_Filter, f_Vin_Filter, f_Vout_Filter, f_Vdc_Filter; // Filtered values
} ADC_Obj;

/************************************************************/
typedef enum
{
    mSENSOR_CALIB = 0xBA,         // Sensor calibration state
    mSTANDSTILL   = 0xBB,         // Standstill / idle state
    mSTOP         = 0xBC,         // Stop state
    mRUN          = 0xBD,         // Normal running state
    mFLT          = 0xBE,         // Fault state
} machine_state;
/************************************************************/
typedef enum
{
	SOCc_50 = 0xAA,
	SOCc_70 = 0xAB,
	SOCc_100 = 0xAC,
} SOCc_state;
/************************************************************/
typedef enum
{
	SOCb_30 = 0xCA,
	SOCb_50 = 0xCB,
	SOCb_70 = 0xCC,
	SOCb_100 = 0xCD,
} SOCb_state;
typedef enum
{
    PI        = 0xDA,             // Pure PI-based control mode
    Fuzzy_SMC = 0xDB,             // Fuzzy + Sliding Mode Control
    Fuzzy_PI  = 0xDC,             // Fuzzy-tuned PI control
    OVB       = 0xDD,             // Other control mode / custom mode
} control_Mode;

typedef enum
{
    State1 = 0xEA,                // No active control yet, pre-charge / initial phase
    State2 = 0xEB,                // P or PI voltage regulation phase
    State3 = 0xEC,                // Full closed-loop control phase
} control_state;
// Machine control
typedef struct
{
    machine_state enum_mState;    // Global machine state
    control_Mode enum_ctrMode;    // Selected control algorithm
    control_state enum_ctrState;  // Current control stage
    SOCc_state enumm_SOCc;        // Supercapacitor SOC state
    SOCb_state enum_SOCb;         // Battery SOC state

    uint16_t ui16_Vdc_ST1;        // Threshold / record for Stage 1 DC-link voltage
    uint16_t ui16_Vdc_ST2;        // Threshold / record for Stage 2 DC-link voltage

    float f_Vdc_SP;               // DC-link voltage reference
    float f_Vdc_fb;               // Measured DC-link voltage feedback
    float f_Vin;                  // Input voltage
    float f_Vout;                 // Output voltage
    float f_Vinc;                 // Compensated / processed Vin
    float f_Voutc;                // Compensated / processed Vout

    float f_Ib_fb;                // Battery current feedback
    float f_Ic_fb;                // Capacitor current feedback

    float f_Ib_sp;                // Battery current setpoint
    float f_Ic_sp;                // Capacitor current setpoint
    float f_It_sp;                // Total current reference from outer loop
    float f_Ift_sp;               // Filtered current reference

    float SOCb;                   // Battery state of charge in percentage
    float SOCc;                   // Capacitor state of charge in percentage
} machine_control;

/***************************************************************/
typedef struct
{
	float L, rL, C;   // para of circuit

} machine_parameter;

/***************************************************************/
typedef struct
{
    float V_b;
    float I_b;
    float SOC_b;
    float I_Bsp;
		
    float V_sc;
    float I_sc;
    float SOC_sc;
		float I_csp;
    float VdcLink;

    uint8_t ID;
		machine_state enum_mState;
		control_state enum_ctrState;
		control_Mode enum_ctrMode;
	
}UART_DATA;
/***************************************************************/
// Define PWM data type
typedef struct {
	float dutyhigh;
	float dutylow;
} PWM;
/***************************************************************/
//usart data
typedef struct
{
	//Buffer
	uint8_t rxBuff[100];
	uint8_t txBuff[100];
	//Header
	uint8_t frameLength, groupID, senderAddr, receiverAddr, messType, dataLength;
	uint16_t messCount;
	//Data payload
	
	//State flow 
	uint8_t STATE;
	uint8_t RxFlag;
	uint8_t rxByte, rxPointer;
	uint16_t timeOut; 
}usartData;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//Controller
/*PI controller*/
void vPI_Init( cPI *pPI, float Ts, float Kp, float TI, float lamda, float Usat);
void vPI_Refresh(cPI *pPI);
float fPI_Run(cPI *pPI, float Ref, float Fb, uint8_t ui8_Mode);
void fPI_UpdateParam(cPI *pPI, float Kp, float Ti);
/*Fuzzy Controller*/
void fuzzyInit(void);
void fuzzifier(float error);
float defuzzifier_Kp(float *Kp);
float defuzzifier_Ti(float *Ti);
/*SMC */
void SMC_Init(SMC *s,float lamda,float k);
float sign(float s);
float SMC_Run(SMC *s,float ref,float fb);
/* Controll loop*/
/*Control Loop using FZ-PI*/
float ControlLoop_FuzzyPI( cPI *pPI, float Ref, float Fb, float ref_in_fz, float fb_ip_fz  );
float ControlLoop_SMC( SMC *smc, float Ref, float Fb, float Vdc, float L, float rL, float Tdk);
void ControlLoop_ST1(machine_control *mctr,PWM *PWM_var);
void ControlLoop_State2( machine_control *mctr,PWM *PWM_var, cPI *pPIV, cPI *pPIIB, LPF *LPFDI);
void ControlLoop_State3_FullPID( machine_control *mctr,PWM *PWM_var, cPI *pPIV, cPI *pPIIB, LPF *LPFDI);
void ControlLoop_State3_FZ( machine_control *mctr,PWM *PWM_var, cPI *pPIV, cPI *pPIIB, LPF *LPFDI);
float SOC( float input, float V_max);
//Low pass filter 
void vLPF_Init(LPF *pLPF, uint16_t ConnerFrqHz, float samplingTime);	//Initialize LPF 
float fLPF_Run(LPF *pLPF, float input);	

//ADC tranform
void vAdc_DataInit(ADC_Obj *pObjt);
void vADC_Data_Handle(ADC_Obj *pObjt);

// Machine
void MachineParameterConfig(machine_parameter *mPara);
void StateMachine();
void StateMachine_Init();
//UART
void vUsart_DataInit(usartData *pData);
void update_data_TX(machine_control *mctr, UART_DATA *data);
void update_data_RX(usartData *pRxData, machine_control *mctr);

HAL_StatusTypeDef USER_UART_TxData(UART_DATA *data);
HAL_StatusTypeDef UART4_StartRxDMA(void);
void USER_UART_RxDataHandle(void);
void UART4_DMA_Init(void);

extern volatile uint8_t uart4_tx_done_flag;
extern volatile uint8_t uart4_rx_done_flag;
//PWM
void PWM_Set(float duty, float freq);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOE
#define BYPASS_Pin GPIO_PIN_3
#define BYPASS_GPIO_Port GPIOA
#define RELAY_Pin GPIO_PIN_4
#define RELAY_GPIO_Port GPIOA
#define ADC1_AI_Pin GPIO_PIN_5
#define ADC1_AI_GPIO_Port GPIOA
#define ADC1_VIN_Pin GPIO_PIN_7
#define ADC1_VIN_GPIO_Port GPIOA
#define ADC1_VOUT_Pin GPIO_PIN_4
#define ADC1_VOUT_GPIO_Port GPIOC
#define ADC1_ISENSE_Pin GPIO_PIN_5
#define ADC1_ISENSE_GPIO_Port GPIOC
#define ADC1_VREF2V5_Pin GPIO_PIN_0
#define ADC1_VREF2V5_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOE
#define RS485_nRW_Pin GPIO_PIN_14
#define RS485_nRW_GPIO_Port GPIOE
#define LORA_NSS_Pin GPIO_PIN_12
#define LORA_NSS_GPIO_Port GPIOB
#define LORA_SCK_Pin GPIO_PIN_13
#define LORA_SCK_GPIO_Port GPIOB
#define LORA_MISO_Pin GPIO_PIN_14
#define LORA_MISO_GPIO_Port GPIOB
#define LORA_MOSI_Pin GPIO_PIN_15
#define LORA_MOSI_GPIO_Port GPIOB
#define LORA_RESET_Pin GPIO_PIN_8
#define LORA_RESET_GPIO_Port GPIOD
#define LORA_DI_Pin GPIO_PIN_9
#define LORA_DI_GPIO_Port GPIOD
#define PWM_LOW_Pin GPIO_PIN_14
#define PWM_LOW_GPIO_Port GPIOD
#define PWM_HIGH_Pin GPIO_PIN_15
#define PWM_HIGH_GPIO_Port GPIOD
#define RGB_Pin GPIO_PIN_15
#define RGB_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
/*---------------------------------------Math constant parameters-------------------------------*/
#define PI 3.14159							// Pi number
#define LMTST0  25
#define LMTST1  30
#define VC_TH   15.0
/*---------------------------------------Set-point----------------------------------------------*/
#define Vref 30     // V
#define Fulse_PWM  400000    //Hz
/*-----------------------------------------lIMIT------------------------------------------------*/
#define MAX_CURRENT_OF_BAT 25
#define MAX_CURRENT_OF_CAP 25
#define MAX_DUTY 0.95
#define MIN_DUTY 0.05
#define ERROR_VOLTAGE 50
/*-----------------------------------------PI------------------------------------------------*/
#define P 0
#define P_I 1
/*-----------------------------------------ADC------------------------------------------------*/
#define NumOfADC_Channel 5
#define AD_MAX_12Bit 4095u
#define AD_MAX_14Bit 16383u
#define AD_MAX_16Bit 65535u
/*----------------------------------------UART-----------------------------------------------*/


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
