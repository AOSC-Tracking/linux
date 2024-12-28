#ifndef __LOONGSON_2K300_ADC_CORE_H__
#define __LOONGSON_2K300_ADC_CORE_H__

typedef enum FunctionalState {
	DISABLE = 0,
	ENABLE,
} FunctionalState;

// ADC register offsets
#define ADC_SR              0x00
#define ADC_CR1             0x04
#define ADC_CR2             0x08
#define ADC_SMPR1           0x0C
#define ADC_SMPR2           0x10
#define ADC_JOFR1           0x14
#define ADC_JOFR2           0x18
#define ADC_JOFR3           0x1C
#define ADC_JOFR4           0x20
#define ADC_HTR             0x24
#define ADC_LTR             0x28
#define ADC_SQR1            0x2C
#define ADC_SQR2            0x30
#define ADC_SQR3            0x34
#define ADC_JSQR            0x38
#define ADC_JDR1            0x3C
#define ADC_JDR2            0x40
#define ADC_JDR3            0x44
#define ADC_JDR4            0x48
#define ADC_DR              0x4C

typedef struct
{
	volatile unsigned int SR;
	volatile unsigned int CR1;
	volatile unsigned int CR2;
	volatile unsigned int SMPR1;
	volatile unsigned int SMPR2;
	volatile unsigned int JOFR1;
	volatile unsigned int JOFR2;
	volatile unsigned int JOFR3;
	volatile unsigned int JOFR4;
	volatile unsigned int HTR;
	volatile unsigned int LTR;
	volatile unsigned int SQR1;
	volatile unsigned int SQR2;
	volatile unsigned int SQR3;
	volatile unsigned int JSQR;
	volatile unsigned int JDR1;
	volatile unsigned int JDR2;
	volatile unsigned int JDR3;
	volatile unsigned int JDR4;
	volatile unsigned int DR;
} adc_reg_map;

/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_adc.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file contains all the functions prototypes for the
*                      ADC firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/

#define   ADC_RCG     (12)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ADC DISCNUM mask */
// #define CR1_DISCNUM_Reset           ((unsigned int)0xFFFF1FFF)

// /* ADC DISCEN mask */
// #define CR1_DISCEN_Set              ((unsigned int)0x00000800)
// #define CR1_DISCEN_Reset            ((unsigned int)0xFFFFF7FF)

// /* ADC JAUTO mask */
// #define CR1_JAUTO_Set               ((unsigned int)0x00000400)
// #define CR1_JAUTO_Reset             ((unsigned int)0xFFFFFBFF)

// /* ADC JDISCEN mask */
// #define CR1_JDISCEN_Set             ((unsigned int)0x00001000)
// #define CR1_JDISCEN_Reset           ((unsigned int)0xFFFFEFFF)

// /* ADC AWDCH mask */
// #define CR1_AWDCH_Reset             ((unsigned int)0xFFFFFFE0)

// /* ADC Analog watchdog enable mode mask */
// #define CR1_AWDMode_Reset           ((unsigned int)0xFF3FFDFF)

// /* CR1 register Mask */
#define CR1_CLEAR_MASK              ((unsigned int)0xFFF0FEFF)

//
#define SR_EOC_OFFSET 0x1
#define SR_J_EOC_OFFSET 0x2

//
#define CR1_EOC_IE_OFFSET 0x5
#define CR1_J_EOC_IE_OFFSET 0x7

/* ADC ADON mask */
#define CR2_ADON_OFFSET             0x0

/* ADC reset */
// #define CR2_ADC_Reset               ((unsigned int)0x80000000)

// /* ADC DMA mask */
// #define CR2_DMA_Set                 ((unsigned int)0x00000100)
// #define CR2_DMA_Reset               ((unsigned int)0xFFFFFEFF)

// /* ADC RSTCAL mask */
// #define CR2_RSTCAL_Set              ((unsigned int)0x00000008)

// /* ADC CAL mask */
// #define CR2_CAL_Set                 ((unsigned int)0x00000004)

// /* ADC SWSTART mask */
// #define CR2_SWSTART_Set             ((unsigned int)0x00400000)

// /* ADC EXTTRIG mask */
// #define CR2_EXTTRIG_Set             ((unsigned int)0x00100000)
// #define CR2_EXTTRIG_Reset           ((unsigned int)0xFFEFFFFF)

/* ADC Software start mask */
#define CR2_EXTTRIG_SW_START_OFFSET 22
#define CR2_EXTTRIG_SW_J_START_OFFSET 21
#define CR2_EXTTRIG_OFFSET 20
#define CR2_J_EXTTRIG_OFFSET 15
// #define CR2_EXTTRIG_SWSTART_Set     ((unsigned int)0x00500000)
// #define CR2_EXTTRIG_SWSTART_Reset   ((unsigned int)0xFFAFFFFF)

/* ADC JEXTSEL mask */
// #define CR2_JEXTSEL_Reset           ((unsigned int)0xFFFF8FFF)

// /* ADC JEXTTRIG mask */
// #define CR2_JEXTTRIG_Set            ((unsigned int)0x00008000)
// #define CR2_JEXTTRIG_Reset          ((unsigned int)0xFFFF7FFF)

// /* ADC JSWSTART mask */
// #define CR2_JSWSTART_Set            ((unsigned int)0x00200000)

// /* ADC injected software start mask */
// #define CR2_JEXTTRIG_JSWSTART_Set   ((unsigned int)0x00208000)
// #define CR2_JEXTTRIG_JSWSTART_Reset ((unsigned int)0xFFDF7FFF)

// /* ADC TSPD mask */
// #define CR2_TSVREFE_Set             ((unsigned int)0x00800000)
// #define CR2_TSVREFE_Reset           ((unsigned int)0xFF7FFFFF)

// /* CR2 register Mask */
#define CR2_CLEAR_MASK              ((unsigned int)0xFFF1F7FD)

// /* ADC SQx mask */
#define SQR3_SQ_MASK 0x1F
#define SQR2_SQ_MASK 0x1F
#define SQR1_SQ_MASK 0x1F

// /* SQR1 register Mask */
#define SQR1_CLEAR_MASK             ((unsigned int)0xFF0FFFFF)

// /* ADC JSQx mask */
// #define JSQR_JSQ_Set                ((unsigned int)0x0000001F)

// /* ADC JL mask */
// #define JSQR_JL_Set                 ((unsigned int)0x00300000)
// #define JSQR_JL_Reset               ((unsigned int)0xFFCFFFFF)

// /* ADC SMPx mask */
#define SMPR1_SMP_MASK 0x7
#define SMPR2_SMP_MASK 0x7

// /* ADC JDRx registers offset */
// #define JDR_Offset                  ((unsigned char)0x28)

// #define DISABLE 0
// #define ENABLE 1

/* Includes ------------------------------------------------------------------*/
// #include "../i2c/ls2k0300_map.h"

/* Exported types ------------------------------------------------------------*/
/* ADC Init structure definition */
typedef struct
{
	unsigned int ADC_Mode;
	int ADC_ScanConvMode;  //CR1 scan
	int ADC_ContinuousConvMode;//CR2 cont
	unsigned int ADC_ExternalTrigConv;//CR2 extsel
	unsigned int ADC_DataAlign;//CR2 align
	unsigned char  ADC_NbrOfChannel;//SQR1 l
	unsigned short ADC_ClkDivider;//CR1 clkdiv
	unsigned char  ADC_JTrigMod;//CR2 jtrigmod
	unsigned char  ADC_ADCEdge;//CR2 adcedge
	unsigned char  ADC_DiffMod;//CR1 diffmod
	unsigned char  ADC_OutPhaseSel;//CR1 ops
	unsigned char  ADC_ClkMask;//CR2 clkmask
	unsigned char ADC_Int_EOC; // CR1 EOC enable ?
	unsigned char ADC_Int_JEOC; // CR1 JEOC enable ?
}adc_init_info;

/* Exported constants --------------------------------------------------------*/

/* ADC dual mode -------------------------------------------------------------*/
#define ADC_Mode_Independent                       ((unsigned int)0x00000000)
#define ADC_Mode_RegInjecSimult                    ((unsigned int)0x00010000)
#define ADC_Mode_RegSimult_AlterTrig               ((unsigned int)0x00020000)
#define ADC_Mode_InjecSimult_FastInterl            ((unsigned int)0x00030000)
#define ADC_Mode_InjecSimult_SlowInterl            ((unsigned int)0x00040000)
#define ADC_Mode_InjecSimult                       ((unsigned int)0x00050000)
#define ADC_Mode_RegSimult                         ((unsigned int)0x00060000)
#define ADC_Mode_FastInterl                        ((unsigned int)0x00070000)
#define ADC_Mode_SlowInterl                        ((unsigned int)0x00080000)
#define ADC_Mode_AlterTrig                         ((unsigned int)0x00090000)

#define IS_ADC_MODE(MODE) (((MODE) == ADC_Mode_Independent) || \
							((MODE) == ADC_Mode_RegInjecSimult) || \
							((MODE) == ADC_Mode_RegSimult_AlterTrig) || \
							((MODE) == ADC_Mode_InjecSimult_FastInterl) || \
							((MODE) == ADC_Mode_InjecSimult_SlowInterl) || \
							((MODE) == ADC_Mode_InjecSimult) || \
							((MODE) == ADC_Mode_RegSimult) || \
							((MODE) == ADC_Mode_FastInterl) || \
							((MODE) == ADC_Mode_SlowInterl) || \
							((MODE) == ADC_Mode_AlterTrig))

/* ADC extrenal trigger sources for regular channels conversion --------------*/
/* for ADC1 and ADC2 */
#define ADC_ExternalTrigConv_T1_CC1                ((unsigned int)0x00000000)
#define ADC_ExternalTrigConv_T1_CC2                ((unsigned int)0x00020000)
#define ADC_ExternalTrigConv_T2_CC2                ((unsigned int)0x00060000)
#define ADC_ExternalTrigConv_T3_TRGO               ((unsigned int)0x00080000)
#define ADC_ExternalTrigConv_T4_CC4                ((unsigned int)0x000A0000)
#define ADC_ExternalTrigConv_Ext_IT11_TIM8_TRGO    ((unsigned int)0x000C0000)
/* for ADC1, ADC2 and ADC3 */
#define ADC_ExternalTrigConv_T1_CC3                ((unsigned int)0x00040000)
#define ADC_ExternalTrigConv_None                  ((unsigned int)0x000E0000)
/* for ADC3 */
#define ADC_ExternalTrigConv_T3_CC1                ((unsigned int)0x00000000)
#define ADC_ExternalTrigConv_T2_CC3                ((unsigned int)0x00020000)
#define ADC_ExternalTrigConv_T8_CC1                ((unsigned int)0x00060000)
#define ADC_ExternalTrigConv_T8_TRGO               ((unsigned int)0x00080000)
#define ADC_ExternalTrigConv_T5_CC1                ((unsigned int)0x000A0000)
#define ADC_ExternalTrigConv_T5_CC3                ((unsigned int)0x000C0000)

#define IS_ADC_EXT_TRIG(REGTRIG) (((REGTRIG) == ADC_ExternalTrigConv_T1_CC1) || \
									((REGTRIG) == ADC_ExternalTrigConv_T1_CC2) || \
									((REGTRIG) == ADC_ExternalTrigConv_T1_CC3) || \
									((REGTRIG) == ADC_ExternalTrigConv_T2_CC2) || \
									((REGTRIG) == ADC_ExternalTrigConv_T3_TRGO) || \
									((REGTRIG) == ADC_ExternalTrigConv_T4_CC4) || \
									((REGTRIG) == ADC_ExternalTrigConv_Ext_IT11_TIM8_TRGO) || \
									((REGTRIG) == ADC_ExternalTrigConv_None) || \
									((REGTRIG) == ADC_ExternalTrigConv_T3_CC1) || \
									((REGTRIG) == ADC_ExternalTrigConv_T2_CC3) || \
									((REGTRIG) == ADC_ExternalTrigConv_T8_CC1) || \
									((REGTRIG) == ADC_ExternalTrigConv_T8_TRGO) || \
									((REGTRIG) == ADC_ExternalTrigConv_T5_CC1) || \
									((REGTRIG) == ADC_ExternalTrigConv_T5_CC3))

/* ADC data align ------------------------------------------------------------*/
#define ADC_DataAlign_Right                        ((unsigned int)0x00000000)
#define ADC_DataAlign_Left                         ((unsigned int)0x00000800)

#define IS_ADC_DATA_ALIGN(ALIGN) (((ALIGN) == ADC_DataAlign_Right) || \
                                  ((ALIGN) == ADC_DataAlign_Left))

/* ADC channels --------------------------------------------------------------*/
#define ADC_Channel_0                               ((unsigned char)0x00)
#define ADC_Channel_1                               ((unsigned char)0x01)
#define ADC_Channel_2                               ((unsigned char)0x02)
#define ADC_Channel_3                               ((unsigned char)0x03)
#define ADC_Channel_4                               ((unsigned char)0x08)
#define ADC_Channel_5                               ((unsigned char)0x09)
#define ADC_Channel_6                               ((unsigned char)0x0a)
#define ADC_Channel_7                               ((unsigned char)0x0b)
#define ADC_Channel_8                               ((unsigned char)0x18)
#define ADC_Channel_9                               ((unsigned char)0x19)
#define ADC_Channel_10                              ((unsigned char)0x1A)
#define ADC_Channel_11                              ((unsigned char)0x1B)
#define ADC_Channel_12                              ((unsigned char)0x1C)
#define ADC_Channel_13                              ((unsigned char)0x1D)
#define ADC_Channel_14                              ((unsigned char)0x1E)
#define ADC_Channel_15                              ((unsigned char)0x1F)
#define ADC_Channel_16                              ((unsigned char)0x10)
#define ADC_Channel_17                              ((unsigned char)0x11)

#define IS_ADC_CHANNEL(CHANNEL) (((CHANNEL) == ADC_Channel_0) || ((CHANNEL) == ADC_Channel_1) || \
								((CHANNEL) == ADC_Channel_2) || ((CHANNEL) == ADC_Channel_3) || \
								((CHANNEL) == ADC_Channel_4) || ((CHANNEL) == ADC_Channel_5) || \
								((CHANNEL) == ADC_Channel_6) || ((CHANNEL) == ADC_Channel_7) || \
								((CHANNEL) == ADC_Channel_8) || ((CHANNEL) == ADC_Channel_9) || \
								((CHANNEL) == ADC_Channel_10) || ((CHANNEL) == ADC_Channel_11) || \
								((CHANNEL) == ADC_Channel_12) || ((CHANNEL) == ADC_Channel_13) || \
								((CHANNEL) == ADC_Channel_14) || ((CHANNEL) == ADC_Channel_15) || \
								((CHANNEL) == ADC_Channel_16) || ((CHANNEL) == ADC_Channel_17))

/* ADC sampling times --------------------------------------------------------*/
#define ADC_SampleTime_1Cycles                    ((unsigned char)0x00)
#define ADC_SampleTime_2Cycles                    ((unsigned char)0x01)
#define ADC_SampleTime_4Cycles                    ((unsigned char)0x02)
#define ADC_SampleTime_8Cycles                    ((unsigned char)0x03)
#define ADC_SampleTime_16Cycles                   ((unsigned char)0x04)
#define ADC_SampleTime_32Cycles                   ((unsigned char)0x05)
#define ADC_SampleTime_64Cycles                   ((unsigned char)0x06)
#define ADC_SampleTime_128Cycles                  ((unsigned char)0x07)

#define IS_ADC_SAMPLE_TIME(TIME) (((TIME) == ADC_SampleTime_1Cycles5)  || \
									((TIME) == ADC_SampleTime_2Cycles5)  || \
									((TIME) == ADC_SampleTime_4Cycles5)  || \
									((TIME) == ADC_SampleTime_8Cycles5)  || \
									((TIME) == ADC_SampleTime_16Cycles5) || \
									((TIME) == ADC_SampleTime_32Cycles5) || \
									((TIME) == ADC_SampleTime_64Cycles5) || \
									((TIME) == ADC_SampleTime_128Cycles5))

/* ADC extrenal trigger sources for injected channels conversion -------------*/
/* For ADC1 and ADC2 */
#define ADC_ExternalTrigInjecConv_T2_TRGO           ((unsigned int)0x00002000)
#define ADC_ExternalTrigInjecConv_T2_CC1            ((unsigned int)0x00003000)
#define ADC_ExternalTrigInjecConv_T3_CC4            ((unsigned int)0x00004000)
#define ADC_ExternalTrigInjecConv_T4_TRGO           ((unsigned int)0x00005000)
#define ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4 ((unsigned int)0x00006000)
/* For ADC1, ADC2 and ADC3 */
#define ADC_ExternalTrigInjecConv_T1_TRGO           ((unsigned int)0x00000000)
#define ADC_ExternalTrigInjecConv_T1_CC4            ((unsigned int)0x00001000)
#define ADC_ExternalTrigInjecConv_None              ((unsigned int)0x00007000)
/* For ADC3 */
#define ADC_ExternalTrigInjecConv_T4_CC3            ((unsigned int)0x00002000)
#define ADC_ExternalTrigInjecConv_T8_CC2            ((unsigned int)0x00003000)
#define ADC_ExternalTrigInjecConv_T8_CC4            ((unsigned int)0x00004000)
#define ADC_ExternalTrigInjecConv_T5_TRGO           ((unsigned int)0x00005000)
#define ADC_ExternalTrigInjecConv_T5_CC4            ((unsigned int)0x00006000)

#define IS_ADC_EXT_INJEC_TRIG(INJTRIG) (((INJTRIG) == ADC_ExternalTrigInjecConv_T1_TRGO) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_T1_CC4) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_T2_TRGO) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_T2_CC1) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_T3_CC4) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_T4_TRGO) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_None) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_T4_CC3) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_T8_CC2) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_T8_CC4) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_T5_TRGO) || \
										((INJTRIG) == ADC_ExternalTrigInjecConv_T5_CC4))

/* ADC injected channel selection --------------------------------------------*/
#define ADC_InjectedChannel_1                       ((unsigned char)0x14)
#define ADC_InjectedChannel_2                       ((unsigned char)0x18)
#define ADC_InjectedChannel_3                       ((unsigned char)0x1C)
#define ADC_InjectedChannel_4                       ((unsigned char)0x20)

#define IS_ADC_INJECTED_CHANNEL(CHANNEL) (((CHANNEL) == ADC_InjectedChannel_1) || \
											((CHANNEL) == ADC_InjectedChannel_2) || \
											((CHANNEL) == ADC_InjectedChannel_3) || \
											((CHANNEL) == ADC_InjectedChannel_4))

/* ADC analog watchdog selection ---------------------------------------------*/
#define ADC_AnalogWatchdog_SingleRegEnable         ((unsigned int)0x00800200)
#define ADC_AnalogWatchdog_SingleInjecEnable       ((unsigned int)0x00400200)
#define ADC_AnalogWatchdog_SingleRegOrInjecEnable  ((unsigned int)0x00C00200)
#define ADC_AnalogWatchdog_AllRegEnable            ((unsigned int)0x00800000)
#define ADC_AnalogWatchdog_AllInjecEnable          ((unsigned int)0x00400000)
#define ADC_AnalogWatchdog_AllRegAllInjecEnable    ((unsigned int)0x00C00000)
#define ADC_AnalogWatchdog_None                    ((unsigned int)0x00000000)

#define IS_ADC_ANALOG_WATCHDOG(WATCHDOG) (((WATCHDOG) == ADC_AnalogWatchdog_SingleRegEnable) || \
											((WATCHDOG) == ADC_AnalogWatchdog_SingleInjecEnable) || \
											((WATCHDOG) == ADC_AnalogWatchdog_SingleRegOrInjecEnable) || \
											((WATCHDOG) == ADC_AnalogWatchdog_AllRegEnable) || \
											((WATCHDOG) == ADC_AnalogWatchdog_AllInjecEnable) || \
											((WATCHDOG) == ADC_AnalogWatchdog_AllRegAllInjecEnable) || \
											((WATCHDOG) == ADC_AnalogWatchdog_None))

/* ADC interrupts definition -------------------------------------------------*/
#define ADC_IT_EOC                                 ((unsigned short)0x0220)
#define ADC_IT_AWD                                 ((unsigned short)0x0140)
#define ADC_IT_JEOC                                ((unsigned short)0x0480)

#define IS_ADC_IT(IT) ((((IT) & (unsigned short)0xF81F) == 0x00) && ((IT) != 0x00))
#define IS_ADC_GET_IT(IT) (((IT) == ADC_IT_EOC) || ((IT) == ADC_IT_AWD) || \
							((IT) == ADC_IT_JEOC))

/* ADC flags definition ------------------------------------------------------*/
#define ADC_FLAG_AWD                               ((unsigned char)0x01)
#define ADC_FLAG_EOC                               ((unsigned char)0x02)
#define ADC_FLAG_JEOC                              ((unsigned char)0x04)
#define ADC_FLAG_JSTRT                             ((unsigned char)0x08)
#define ADC_FLAG_STRT                              ((unsigned char)0x10)

#define IS_ADC_CLEAR_FLAG(FLAG) ((((FLAG) & (unsigned char)0xE0) == 0x00) && ((FLAG) != 0x00))
#define IS_ADC_GET_FLAG(FLAG) (((FLAG) == ADC_FLAG_AWD) || ((FLAG) == ADC_FLAG_EOC) || \
								((FLAG) == ADC_FLAG_JEOC) || ((FLAG)== ADC_FLAG_JSTRT) || \
								((FLAG) == ADC_FLAG_STRT))

/* ADC thresholds ------------------------------------------------------------*/
#define IS_ADC_THRESHOLD(THRESHOLD) ((THRESHOLD) <= 0xFFF)

/* ADC injected offset -------------------------------------------------------*/
#define IS_ADC_OFFSET(OFFSET) ((OFFSET) <= 0xFFF)

/* ADC injected length -------------------------------------------------------*/
#define IS_ADC_INJECTED_LENGTH(LENGTH) (((LENGTH) >= 0x1) && ((LENGTH) <= 0x4))

/* ADC injected rank ---------------------------------------------------------*/
#define IS_ADC_INJECTED_RANK(RANK) (((RANK) >= 0x1) && ((RANK) <= 0x4))

/* ADC regular length --------------------------------------------------------*/
#define IS_ADC_REGULAR_LENGTH(LENGTH) (((LENGTH) >= 0x1) && ((LENGTH) <= 0x10))

/* ADC regular rank ----------------------------------------------------------*/
#define IS_ADC_REGULAR_RANK(RANK) (((RANK) >= 0x1) && ((RANK) <= 0x10))

/* ADC regular discontinuous mode number -------------------------------------*/
#define IS_ADC_REGULAR_DISC_NUMBER(NUMBER) (((NUMBER) >= 0x1) && ((NUMBER) <= 0x8))

//Loongson Feature
#define RCH0  ADC_Channel_0
#define RCH1  ADC_Channel_1
#define RCH2  ADC_Channel_2
#define RCH3  ADC_Channel_3
#define RCH4  ADC_Channel_4
#define RCH5  ADC_Channel_5
#define RCH6  ADC_Channel_6
#define RCH7  ADC_Channel_7

//Loongson Feature
#define JCH0  ADC_Channel_0
#define JCH1  ADC_Channel_1
#define JCH2  ADC_Channel_2
#define JCH3  ADC_Channel_3

void adc_init(adc_reg_map* ADCx, adc_init_info* ADC_InitStruct);
void adc_struct_init(adc_init_info* ADC_InitStruct);
void ADC_RegularChannelConfig(adc_reg_map* ADCx, unsigned char ADC_Channel, unsigned char Rank, unsigned char ADC_SampleTime);
void adc_dev_enable(adc_reg_map* ADCx, FunctionalState state);
void adc_software_start_conv_trigger(adc_reg_map* ADCx, FunctionalState state, unsigned char J_channel);
void adc_eoc_check_conv_end(adc_reg_map* ADCx);

#endif /* __LOONGSON_2K300_ADC_CORE_H__ */
