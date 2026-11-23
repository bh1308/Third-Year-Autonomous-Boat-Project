#include "YEEP144.h"
#include "YEEP144_Pinout.h"

#include "stm32h5xx_hal_adc.h"
#include "stm32h5xx_hal_gpio.h"

#define ADC_TIMEOUT 1000

ADC_HandleTypeDef hadc1;

/**
 * Initialise the conductivity sensor. ADC1, along with 2 GPIO, MCU Pin PA6, ADC12_INP3, and is used to measure the resistance of water between two probes.
 * The resistance is calculated using a known resistor in series with the water, and measuring the voltage across the water. 
 * The resistance of the water is then calculated using Ohm's law.
 * The conductivity of the water can then be calculated using the formula: conductivity = 1 /resistance. The conductivity is then returned in Siemens (S).
 * This function must be called before any other conductivity functions are called.
 */
void YEEP144_Conductivity_Init(void){

    // Setup and enable the ADC peripheral clock
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
    PeriphClkInitStruct.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HSI; // HSI = 64MHz input clock
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK){
      //Error_Handler();
    }

    __HAL_RCC_ADC_CLK_ENABLE();

    // Configure and initialise the ADC peripheral

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1; // No clock division = 64MHz
    hadc1.Init.Resolution = ADC_RESOLUTION_12B; // 12-bit resolution (maximum)
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1; // 1 channel in use
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // Conversion triggered by software
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE; // No DMA
    hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = ENABLE; // Enable oversampling
    hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256; // 256x oversampling gets us 4 extra bits of resolution
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8; // Shift by 8 to get us back to 12-bit output
    hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK){
    //Error_Handler();
    }

    // Initialise the GPIO used
    YEEP144_GPIO_Init(Cond_PORT, Cond_PIN, GPIO_MODE_ANALOG);
}


/**
 * Gets the current value of potentiometer 2.
 * Returns a 12-bit value in the range 0 - 4095.
 * This blocks until the conversion is complete, ~1ms
 */
uint16_t YEEP144_Conductivity_GetReading(void){
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    // With a 64MHz clock and 256x oversampling, 247.5 cycles sample gives us a conversion time of ~1ms
    // or a maximum sample rate of ~1kHz
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
    //Error_Handler();
    }

    // Start the conversion
    HAL_ADC_Start(&hadc1);

    // Wait until the conversion is finished
    HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT);

    // Grab the converted value now conversion is finished
    uint16_t adcval = HAL_ADC_GetValue(&hadc1);

    // Stop the ADC
    HAL_ADC_Stop(&hadc1);

    return adcval;
}
