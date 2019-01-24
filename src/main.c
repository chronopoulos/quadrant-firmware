#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

// globals

ADC_HandleTypeDef hadc;

uint8_t USB_BUF[17];
uint32_t adcResult = 0;

uint16_t IR_PINS_ARRAY[4] = {GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_7};
uint16_t IR_PINS_ALL = GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_7;

// forward declarations
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);

int main(void) {

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC_Init();
    MX_USB_DEVICE_Init();

    uint32_t i;

    // initialize USB_BUF to zeros,newline
    for (i=0; i<16; i++) {
        USB_BUF[i] = 0;
    }
    USB_BUF[16] = 10; // newline

    // main loop
    while (1) {

        // measure signal
        for (i=0; i<4; i++) {

            HAL_GPIO_WritePin(GPIOA, IR_PINS_ARRAY[i], GPIO_PIN_SET);

            HAL_ADC_Start(&hadc);
            if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK) {
                _Error_Handler(__FILE__, __LINE__);
            }
            adcResult = 4096 - HAL_ADC_GetValue(&hadc);

            USB_BUF[2*i] = adcResult >> 8;
            USB_BUF[2*i+1] = adcResult >> 0;

            HAL_GPIO_WritePin(GPIOA, IR_PINS_ARRAY[i], GPIO_PIN_RESET);

        }

        // measure background
        for (i=0; i<4; i++) {

            HAL_ADC_Start(&hadc);
            if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK) {
                _Error_Handler(__FILE__, __LINE__);
            }
            adcResult = 4096 - HAL_ADC_GetValue(&hadc);

            USB_BUF[8 + 2*i] = adcResult >> 8;
            USB_BUF[8 + 2*i+1] = adcResult >> 0;

        }

        // transmit over USB
        CDC_Transmit_FS(&USB_BUF, 17);

        // throttle
        HAL_Delay(30);

    }

}

void SystemClock_Config(void) {

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure the Systick interrupt time
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    // Configure the Systick
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}


static void MX_ADC_Init(void) {

    // ADC config

    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    if (HAL_ADC_Init(&hadc) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // channel config

    ADC_ChannelConfTypeDef sConfig;

    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    sConfig.Channel = ADC_CHANNEL_0;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfig.Channel = ADC_CHANNEL_2;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfig.Channel = ADC_CHANNEL_4; if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfig.Channel = ADC_CHANNEL_6;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

}

static void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pins: PA1 PA3 PA5 PA7
    GPIO_InitStruct.Pin = IR_PINS_ALL;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure GPIO pin: PB1
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, IR_PINS_ALL, GPIO_PIN_RESET);

}

void _Error_Handler(char *file, int line) {

    while(1) {

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(50);

    }

}

