#include <stm32f30x_gpio.h>
#include <stm32f30x_rcc.h>
#include <stm32f30x_adc.h>
#include <stdio.h>
#include <ADCF3.h>

uint16_t AD_value;
uint16_t V25 = 1933;// when V25=1.41V at ref 3.3V
float Avg_Slope = 5.33; //when avg_slope=4.3mV/C at ref 3.3V
int16_t TemperatureC;

void ADC1_config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	volatile int i;
	uint16_t CalibrationValue;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;  
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_SampleTime_2Cycles5;
	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
  
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   //
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  //
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	for (i = 0; i < 10000; i++)
		;
	//wake up temperature sensor
	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) != RESET)
		;
	CalibrationValue = ADC_GetCalibrationValue(ADC1);
 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vbat, 1, ADC_SampleTime_61Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_61Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_61Cycles5);
	/* Insert delay equal to 10 us */
	// Enable VBAT channel
	ADC_VbatCmd(ADC1, ENABLE);
	// Enable Vref
	ADC_VrefintCmd(ADC1, ENABLE);
	// Wake up temp sensor
	ADC_TempSensorCmd(ADC1, ENABLE);
	// Start ADC
	ADC_Cmd(ADC1, ENABLE);
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY))
		;
	ADC_StartConversion(ADC1);  
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY) == RESET)
		;
}

uint16_t readADC1(void)
{
	ADC_StartConversion(ADC1);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
		;
	AD_value = ADC_GetConversionValue(ADC1);
	return AD_value;
}

uint16_t readTemp(void)
{
	uint16_t adcData;
	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_61Cycles5);
	adcData = readADC1();
	printf("%03X\n", adcData); // 0x6DB on my test
	TemperatureC = (int16_t)(((V25 - adcData) / Avg_Slope) + 25);
	printf("Temperature: %d%cC\r\n", TemperatureC, 176);
	return adcData;
}

uint16_t readVBat(void)
{
	uint16_t adcData;
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vbat, 1, ADC_SampleTime_61Cycles5);
	adcData = readADC1();
	return adcData;
}

uint16_t readintRef(void)
{
	uint16_t adcData;
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_61Cycles5);
	adcData = readADC1();
	return adcData;
}

uint16_t readVinput(void)
{
	uint16_t adcData;
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_61Cycles5);
	adcData = readADC1();
	return adcData;
}
