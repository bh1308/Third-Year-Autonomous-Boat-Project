#include "YEEP144.h"
#include "YEEP144_Pinout.h"
#include "YEEP144_DebugUART.h"
#include "stm32h5xx_hal_gpio.h"
#include "stm32h5xx_hal.h"



/**
 * Initialise the DS18B20 sensor. MCU Pin PC0, and is used to measure the temperature of the water.
 * The temperature is calculated using a known resistor in series with the water, and measuring the voltage across the water. 
 * The temperature of the water is then calculated using Ohm's law.
 */


/* DS18B20 commands */
#define DS18B20_CMD_SKIP_ROM       0xCC
#define DS18B20_CMD_CONVERT_T      0x44
#define DS18B20_CMD_READ_SCRATCH   0xBE

/* DS18B20 temperature conversion time (12-bit resolution) */
#define DS18B20_CONVERSION_TIME_MS  750


static void DS18B20_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DS18B20_Delay_us(uint32_t us)
{
    uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    uint32_t cycles = us * cycles_per_us;
    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < cycles)
    {
        // Wait
    }
}

/* ---------------------------------------------------------
   GPIO configuration
   --------------------------------------------------------- */

static void DS18B20_Pin_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DS18B20_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct);

    // Release the 1-Wire bus
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
}

/* Pull the bus LOW */
static void DS18B20_Pin_Low(void)
{
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
}

/* Release the bus (pull-up resistor brings it HIGH) */
static void DS18B20_Pin_Release(void)
{
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
}

/* Read the current bus state */
static uint8_t DS18B20_Pin_Read(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN);
}

/* ---------------------------------------------------------
   1-Wire reset and presence detection
   --------------------------------------------------------- */

static uint8_t DS18B20_Reset(void)
{
    uint8_t presence;

    DS18B20_Pin_Low();
    DS18B20_Delay_us(480);

    DS18B20_Pin_Release();
    DS18B20_Delay_us(70);

    // Sensor pulls the bus LOW to indicate presence
    presence = (DS18B20_Pin_Read() == GPIO_PIN_RESET);

    DS18B20_Delay_us(410);

    return presence;
}

/* ---------------------------------------------------------
   Write one bit
   --------------------------------------------------------- */

static void DS18B20_WriteBit(uint8_t bit)
{
    DS18B20_Pin_Low();

    if (bit)
    {
        // Write 1
        DS18B20_Delay_us(6);
        DS18B20_Pin_Release();
        DS18B20_Delay_us(64);
    }
    else
    {
        // Write 0
        DS18B20_Delay_us(60);
        DS18B20_Pin_Release();
        DS18B20_Delay_us(10);
    }
}

/* ---------------------------------------------------------
   Read one bit
   --------------------------------------------------------- */

static uint8_t DS18B20_ReadBit(void)
{
    uint8_t bit;

    DS18B20_Pin_Low();
    DS18B20_Delay_us(3);

    DS18B20_Pin_Release();
    DS18B20_Delay_us(10);

    bit = DS18B20_Pin_Read();

    DS18B20_Delay_us(53);

    return bit;
}

/* ---------------------------------------------------------
   Write one byte (LSB first)
   --------------------------------------------------------- */

static void DS18B20_WriteByte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_WriteBit(data & 0x01);
        data >>= 1;
    }
}

/* ---------------------------------------------------------
   Read one byte (LSB first)
   --------------------------------------------------------- */

static uint8_t DS18B20_ReadByte(void)
{
    uint8_t data = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        data >>= 1;

        if (DS18B20_ReadBit())
        {
            data |= 0x80;
        }
    }

    return data;
}

/* ---------------------------------------------------------
   CRC-8 calculation (DS18B20 polynomial)
   --------------------------------------------------------- */

static uint8_t DS18B20_CRC8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t inbyte = data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ inbyte) & 0x01;

            crc >>= 1;

            if (mix)
            {
                crc ^= 0x8C;
            }

            inbyte >>= 1;
        }
    }

    return crc;
}

/* ---------------------------------------------------------
   Public initialisation function
   --------------------------------------------------------- */

void YEEP144_DS18B20_Init(void)
{
    DS18B20_DWT_Init();
    DS18B20_Pin_Init();
}

/* ---------------------------------------------------------
   Public temperature reading function
   --------------------------------------------------------- */

float YEEP144_DS18B20_GetTemperature(void)
{
    uint8_t scratchpad[9];
    uint16_t raw;
    int16_t signed_raw;
    float temperature;

    /* Check if the sensor is connected */
    if (!DS18B20_Reset())
    {
        return NAN;
    }

    /* Start temperature conversion */
    DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(DS18B20_CMD_CONVERT_T);

    /* Wait for conversion to complete (12-bit resolution) */
    HAL_Delay(DS18B20_CONVERSION_TIME_MS);

    /* Reset bus and request temperature data */
    if (!DS18B20_Reset())
    {
        return NAN;
    }

    DS18B20_WriteByte(DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(DS18B20_CMD_READ_SCRATCH);

    /* Read all 9 scratchpad bytes */
    for (uint8_t i = 0; i < 9; i++)
    {
        scratchpad[i] = DS18B20_ReadByte();
    }

    /* Validate CRC */
    if (DS18B20_CRC8(scratchpad, 8) != scratchpad[8])
    {
        return NAN;
    }

    /* Combine temperature bytes */
    raw = ((uint16_t)scratchpad[1] << 8) | scratchpad[0];

    /* Convert signed temperature value */
    signed_raw = (int16_t)raw;

    /* DS18B20: 1 bit = 0.0625 degrees Celsius */
    temperature = (float)signed_raw / 16.0f;

    return temperature;
}
