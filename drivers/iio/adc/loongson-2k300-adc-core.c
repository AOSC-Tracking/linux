#include <linux/module.h>
#include <linux/delay.h>
#include "loongson-2k300-adc-core.h"

#define ADC_DATA_MASK       0x0FFF

/*******************************************************************************
* Function Name  : adc_init
* Description    : Initializes the ADCx peripheral according to the specified parameters
*                  in the ADC_InitStruct.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_InitStruct: pointer to an adc_init_info structure that
*                    contains the configuration information for the specified
*                    ADC peripheral.
* Output         : None
* Return         : None
******************************************************************************/
void adc_init(adc_reg_map* ADCx, adc_init_info* ADC_InitStruct)
{
	unsigned int temp_val_1 = 0;
	unsigned int temp_val_2 = 0;

	/*---------------------------- ADCx CR1 Configuration -----------------*/
	/* Get the ADCx CR1 value */
	temp_val_1 = ADCx->CR1;
	/* Clear DUALMOD and SCAN bits */
	temp_val_1 &= CR1_CLEAR_MASK;
	/* Configure ADCx: Dual mode and scan conversion mode */
	/* Set DUALMOD bits according to ADC_Mode value */
	/* Set SCAN bit according to ADC_ScanConvMode value */
	temp_val_1 |= (unsigned int)(ADC_InitStruct->ADC_Mode | ((unsigned int)ADC_InitStruct->ADC_ScanConvMode << 8) |
				((unsigned int)((ADC_InitStruct->ADC_ClkDivider)&0x3f) << 24)  |
				((unsigned int)ADC_InitStruct->ADC_DiffMod << 20) |
				((unsigned int)ADC_InitStruct->ADC_OutPhaseSel << 30)); //yg
	temp_val_1 |= 1 << CR1_EOC_IE_OFFSET;
	temp_val_1 ^= 1 << CR1_EOC_IE_OFFSET;
	temp_val_1 |= ADC_InitStruct->ADC_Int_EOC << CR1_EOC_IE_OFFSET;
	temp_val_1 |= 1 << CR1_J_EOC_IE_OFFSET;
	temp_val_1 ^= 1 << CR1_J_EOC_IE_OFFSET;
	temp_val_1 |= ADC_InitStruct->ADC_Int_JEOC << CR1_J_EOC_IE_OFFSET;
	/* Write to ADCx CR1 */
	ADCx->CR1 = temp_val_1;

	/*---------------------------- ADCx CR2 Configuration -----------------*/
	/* Get the ADCx CR2 value */
	temp_val_1 = ADCx->CR2;
	/* Clear CONT, ALIGN and EXTSEL bits */
	temp_val_1 &= CR2_CLEAR_MASK;
	/* Configure ADCx: external trigger event and continuous conversion mode */
	/* Set ALIGN bit according to ADC_DataAlign value */
	/* Set EXTSEL bits according to ADC_ExternalTrigConv value */
	/* Set CONT bit according to ADC_ContinuousConvMode value */
	temp_val_1 |= (unsigned int)(ADC_InitStruct->ADC_DataAlign | ADC_InitStruct->ADC_ExternalTrigConv |
				((unsigned int)ADC_InitStruct->ADC_ContinuousConvMode << 1) |
				((unsigned int)ADC_InitStruct->ADC_JTrigMod << 24) | //yg
				((unsigned int)(((ADC_InitStruct->ADC_ClkDivider)>>6)&0xf) << 26)  |
				((unsigned int)ADC_InitStruct->ADC_ADCEdge << 30)  |
				((unsigned int)ADC_InitStruct->ADC_ClkMask << 31)) ; //yg
	/* Write to ADCx CR2 */
	ADCx->CR2 = temp_val_1;

	/*---------------------------- ADCx SQR1 Configuration -----------------*/
	/* Get the ADCx SQR1 value */
	temp_val_1 = ADCx->SQR1;
	/* Clear L bits */
	temp_val_1 &= SQR1_CLEAR_MASK;
	/* Configure ADCx: regular channel sequence length */
	/* Set L bits according to ADC_NbrOfChannel value */
	temp_val_2 |= (ADC_InitStruct->ADC_NbrOfChannel - 1);
	temp_val_1 |= ((unsigned int)temp_val_2 << 20);
	/* Write to ADCx SQR1 */
	ADCx->SQR1 = temp_val_1;
}

/*******************************************************************************
* Function Name  : adc_struct_init
* Description    : Fills each ADC_InitStruct member with its default value.
* Input          : ADC_InitStruct : pointer to an adc_init_info structure
*                  which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void adc_struct_init(adc_init_info* ADC_InitStruct)
{
	/* Reset ADC init structure parameters values */
	/* Initialize the ADC_Mode member */
	ADC_InitStruct->ADC_Mode = ADC_Mode_Independent;
	/* initialize the ADC_ScanConvMode member */
	ADC_InitStruct->ADC_ScanConvMode = DISABLE;
	/* Initialize the ADC_ContinuousConvMode member */
	ADC_InitStruct->ADC_ContinuousConvMode = DISABLE;
	/* Initialize the ADC_ExternalTrigConv member */
	ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	/* Initialize the ADC_DataAlign member */
	ADC_InitStruct->ADC_DataAlign = ADC_DataAlign_Right;
	/* Initialize the ADC_NbrOfChannel member */
	ADC_InitStruct->ADC_NbrOfChannel = 1;
	/* Initialize the ADC_ClkDivider member */
	ADC_InitStruct->ADC_ClkDivider = 0xff;
	/* Initialize the ADC_JTrigMod member */
	ADC_InitStruct->ADC_JTrigMod = 0;
	/* Initialize the ADC_ADCEdge member */
	ADC_InitStruct->ADC_ADCEdge = 0;
	/* Initialize the ADC_DIFFMOD member */
	ADC_InitStruct->ADC_DiffMod = 0;
	/* Initialize the ADC_OutPhaseSel member */
	ADC_InitStruct->ADC_OutPhaseSel = 0;
	/* Initialize the ADC_ClkMask member */
	ADC_InitStruct->ADC_ClkMask = 0;
	/* Initialize CR1 EOC disable */
	ADC_InitStruct->ADC_Int_EOC = DISABLE;
	/* Initialize CR1 EOC disable */
	ADC_InitStruct->ADC_Int_JEOC = DISABLE;
}

void ADC_RegularChannelConfig(adc_reg_map* ADCx, unsigned char ADC_Channel, unsigned char Rank, unsigned char ADC_SampleTime)
{
	unsigned int temp_val_1 = 0, temp_val_2 = 0;

	/* if ADC_Channel_10 ... ADC_Channel_17 is selected */
	if (ADC_Channel > ADC_Channel_9) {
		/* Get the old register value */
		temp_val_1 = ADCx->SMPR1;
		/* Calculate the mask to clear */
		temp_val_2 = SMPR1_SMP_MASK << (3 * (ADC_Channel - 10));
		/* Clear the old discontinuous mode channel count */
		temp_val_1 &= ~temp_val_2;
		/* Calculate the mask to set */
		temp_val_2 = (unsigned int)ADC_SampleTime << (3 * (ADC_Channel - 10));
		/* Set the discontinuous mode channel count */
		temp_val_1 |= temp_val_2;
		/* Store the new register value */
		ADCx->SMPR1 = temp_val_1;
	} else { /* ADC_Channel include in ADC_Channel_[0..9] */
		/* Get the old register value */
		temp_val_1 = ADCx->SMPR2;
		/* Calculate the mask to clear */
		temp_val_2 = SMPR2_SMP_MASK << (3 * ADC_Channel);
		/* Clear the old discontinuous mode channel count */
		temp_val_1 &= ~temp_val_2;
		/* Calculate the mask to set */
		temp_val_2 = (unsigned int)ADC_SampleTime << (3 * ADC_Channel);
		/* Set the discontinuous mode channel count */
		temp_val_1 |= temp_val_2;
		/* Store the new register value */
		ADCx->SMPR2 = temp_val_1;
	}
	/* For Rank 1 to 6 */
	if (Rank < 7) {
		/* Get the old register value */
		temp_val_1 = ADCx->SQR3;
		/* Calculate the mask to clear */
		temp_val_2 = SQR3_SQ_MASK << (5 * (Rank - 1));
		/* Clear the old SQx bits for the selected rank */
		temp_val_1 &= ~temp_val_2;
		/* Calculate the mask to set */
		temp_val_2 = (unsigned int)ADC_Channel << (5 * (Rank - 1));
		/* Set the SQx bits for the selected rank */
		temp_val_1 |= temp_val_2;
		/* Store the new register value */
		ADCx->SQR3 = temp_val_1;
	} else if (Rank < 13) { /* For Rank 7 to 12 */
		/* Get the old register value */
		temp_val_1 = ADCx->SQR2;
		/* Calculate the mask to clear */
		temp_val_2 = SQR2_SQ_MASK << (5 * (Rank - 7));
		/* Clear the old SQx bits for the selected rank */
		temp_val_1 &= ~temp_val_2;
		/* Calculate the mask to set */
		temp_val_2 = (unsigned int)ADC_Channel << (5 * (Rank - 7));
		/* Set the SQx bits for the selected rank */
		temp_val_1 |= temp_val_2;
		/* Store the new register value */
		ADCx->SQR2 = temp_val_1;
	} else { /* For Rank 13 to 16 */
		/* Get the old register value */
		temp_val_1 = ADCx->SQR1;
		/* Calculate the mask to clear */
		temp_val_2 = SQR1_SQ_MASK << (5 * (Rank - 13));
		/* Clear the old SQx bits for the selected rank */
		temp_val_1 &= ~temp_val_2;
		/* Calculate the mask to set */
		temp_val_2 = (unsigned int)ADC_Channel << (5 * (Rank - 13));
		/* Set the SQx bits for the selected rank */
		temp_val_1 |= temp_val_2;
		/* Store the new register value */
		ADCx->SQR1 = temp_val_1;
	}
}

/*******************************************************************************
* Function Name  : adc_dev_enable
* Description    : Enables or disables the specified ADC peripheral.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - state: new state of the ADCx peripheral. This parameter
*                    can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void adc_dev_enable(adc_reg_map* ADCx, FunctionalState state)
{
	unsigned int temp;

	if (state == ENABLE) {
		temp = ADCx->CR2;
		temp |= 1 << CR2_ADON_OFFSET;
		temp ^= 1 << CR2_ADON_OFFSET;
		temp |= (state << CR2_ADON_OFFSET);
		ADCx->CR2 = temp;

		// 触发复位校准和AD校准
		ADCx->CR2 = temp;
		temp = ADCx->CR2;
		temp |= (state << 2) | (state << 3);
		ADCx->CR2 = temp;
#if 0
		while (1) {
			temp = ADCx->CR2;
			temp &= 1 << 2;
			if (!temp)
				break;
		}
#else
		ndelay(1000000);
#endif
	} else {
		temp = ADCx->CR2;
		temp |= 1 << CR2_ADON_OFFSET;
		temp ^= 1 << CR2_ADON_OFFSET;
		ADCx->CR2 = temp;
	}
}

/*******************************************************************************
* Function Name  : adc_software_start_conv_trigger
* Description    : Enables or disables the selected ADC software start conversion .
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - state: new state of the selected ADC software start conversion.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void adc_software_start_conv_trigger(adc_reg_map* ADCx, FunctionalState state, unsigned char J_channel)
{
	unsigned int temp;
	unsigned char EOC_offset;
	unsigned char start_offset;
	unsigned char exttrig_offset;

	EOC_offset = J_channel ? SR_J_EOC_OFFSET : SR_EOC_OFFSET;
	start_offset = J_channel ? CR2_EXTTRIG_SW_J_START_OFFSET : CR2_EXTTRIG_SW_START_OFFSET;
	exttrig_offset = J_channel ? CR2_J_EXTTRIG_OFFSET : CR2_EXTTRIG_OFFSET;

	if (state == ENABLE) {
		temp = ADCx->CR2;
		temp |= 1 << exttrig_offset;
		ADCx->CR2 = temp;
		// 先把 EOC 清除
		temp = ADCx->SR;
		temp |= 1 << EOC_offset;
		temp ^= 1 << EOC_offset;
		ADCx->SR = temp;
		ADCx->CR2 |= 1 << start_offset;
	} else {
		/* Disable the selected ADC conversion on external event and stop the selected ADC conversion */
		temp = ADCx->CR2;
		temp |= 1 << start_offset;
		temp ^= 1 << start_offset;
		ADCx->CR2 = temp;
	}
}

void adc_eoc_check_conv_end(adc_reg_map* ADCx)
{
	int max_loop;
	unsigned int temp;

	max_loop = 0;
	while (1) {
		temp = ADCx->SR;
		temp &= 1 << 1;
		ndelay(10);
		++max_loop;
		if (temp == 2)
			break;
		if (max_loop == 10000) {
			pr_info("adc_eoc_check_conv_end eoc wait timeout\n");
			break;
		}
	}
}
