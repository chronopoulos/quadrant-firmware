#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

#define ADDR7_DEFAULT 0x29
#define ADDR7_NORTH 0x2a
#define ADDR7_EAST 0x2b
#define ADDR7_SOUTH 0x2c
#define ADDR7_WEST 0x2d
#define MODEL_ID 0xb4
#define THRESH 180

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

void write8(uint8_t devAddr7, uint16_t regAddr, uint8_t val) {

    unsigned char buf[3];

    buf[0] = regAddr >> 8;
    buf[1] = regAddr & 0xff;
    buf[2] = val;

    HAL_I2C_Master_Transmit(&hi2c1, (devAddr7 << 1), buf, 3, 100);

}

uint8_t read8(uint8_t devAddr7, uint16_t addr) {

    uint8_t buf[2];

    buf[0] = addr >> 8;
    buf[1] = addr & 0xff;

    HAL_I2C_Master_Transmit(&hi2c1, (devAddr7 << 1), buf, 2, 100);
    HAL_I2C_Master_Receive(&hi2c1, (devAddr7 << 1), buf, 1, 100);

    return buf[0];

}

void loadSettings(uint8_t devAddr7) {

    // private settings from page 24 of app note
    write8(devAddr7, 0x0207, 0x01);
    write8(devAddr7, 0x0208, 0x01);
    write8(devAddr7, 0x0096, 0x00);
    write8(devAddr7, 0x0097, 0xfd);
    write8(devAddr7, 0x00e3, 0x00);
    write8(devAddr7, 0x00e4, 0x04);
    write8(devAddr7, 0x00e5, 0x02);
    write8(devAddr7, 0x00e6, 0x01);
    write8(devAddr7, 0x00e7, 0x03);
    write8(devAddr7, 0x00f5, 0x02);
    write8(devAddr7, 0x00d9, 0x05);
    write8(devAddr7, 0x00db, 0xce);
    write8(devAddr7, 0x00dc, 0x03);
    write8(devAddr7, 0x00dd, 0xf8);
    write8(devAddr7, 0x009f, 0x00);
    write8(devAddr7, 0x00a3, 0x3c);
    write8(devAddr7, 0x00b7, 0x00);
    write8(devAddr7, 0x00bb, 0x3c);
    write8(devAddr7, 0x00b2, 0x09);
    write8(devAddr7, 0x00ca, 0x09);
    write8(devAddr7, 0x0198, 0x01);
    write8(devAddr7, 0x01b0, 0x17);
    write8(devAddr7, 0x01ad, 0x00);
    write8(devAddr7, 0x00ff, 0x05);
    write8(devAddr7, 0x0100, 0x05);
    write8(devAddr7, 0x0199, 0x05);
    write8(devAddr7, 0x01a6, 0x1b);
    write8(devAddr7, 0x01ac, 0x3e);
    write8(devAddr7, 0x01a7, 0x1f);
    write8(devAddr7, 0x0030, 0x00);

    // Recommended : Public registers - See data sheet for more detail
    write8(devAddr7, 0x0011, 0x10);       // Enables polling for 'New Sample ready'
                                // when measurement completes
    write8(devAddr7, 0x003f, 0x46);       // Sets the light and dark gain (upper
                                // nibble). Dark gain should not be
                                // changed.
    write8(devAddr7, 0x0031, 0xFF);       // sets the # of range measurements after
                                // which auto calibration of system is
                                // performed
    write8(devAddr7, 0x0040, 0x63);       // Set ALS integration time to 100ms
    write8(devAddr7, 0x002e, 0x01);       // perform a single temperature calibration
                                // of the ranging sensor

    // Optional: Public registers - See data sheet for more detail
    write8(devAddr7, 0x003e, 0x31);       // Set default ALS inter-measurement period
                                // to 500ms
    write8(devAddr7, 0x0014, 0x24);       // Configures interrupt on 'New Sample
                                // Ready threshold event'

    // timing parameters
    write8(devAddr7, 0x01c, 20);    // set max convergence time to 20 ms
    write8(devAddr7, 0x10a, 48);    // Set the averaging sample period to 4.3 ms
    write8(devAddr7, 0x001b, 2); // Set range intermeasurement period to 30 ms

}

void checkModelID(uint8_t devAddr7) {

    // check model ID
    if (read8(devAddr7, 0x00) != MODEL_ID) {
        _Error_Handler(__FILE__, __LINE__);
    }

}

void bringUpSensors(void) {

    // north
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(100); // TODO check for ready state
    write8(ADDR7_DEFAULT, 0x212, ADDR7_NORTH);
    loadSettings(ADDR7_NORTH);
    checkModelID(ADDR7_NORTH);

    // east
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(100); // TODO check for ready state
    write8(ADDR7_DEFAULT, 0x212, ADDR7_EAST);
    loadSettings(ADDR7_EAST);
    checkModelID(ADDR7_EAST);

    // south
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(100); // TODO check for ready state
    write8(ADDR7_DEFAULT, 0x212, ADDR7_SOUTH);
    loadSettings(ADDR7_SOUTH);
    checkModelID(ADDR7_SOUTH);

    // west
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(100); // TODO check for ready state
    write8(ADDR7_DEFAULT, 0x212, ADDR7_WEST);
    loadSettings(ADDR7_WEST);
    checkModelID(ADDR7_WEST);

}

uint8_t measureRange(uint8_t devAddr7) {

    uint8_t result;

    while ( !(read8(devAddr7, 0x4d) & 0x01) ) { // read from RESULT_RANGE_STATUS
        // spin
    }

    write8(devAddr7, 0x18, 0x01); // write 0x01 to SYSRANGE_START

    while ( !(read8(devAddr7, 0x4f) & 0x04) ) { // read from INTERRUPT_STATUS_GPIO
        // spin
    }

    result = read8(devAddr7, 0x62); // read from RESULT_RANGE_VAL

    write8(devAddr7, 0x15, 0x07); // write 0x07 to SYSTEM_INTERRUPT_CLEAR

    return result;

}

uint8_t readRangeContinuous(uint8_t devAddr7) {

    uint8_t result;

    while ( !(read8(devAddr7, 0x4f) & 0x04) ) { // read from INTERRUPT_STATUS_GPIO
        // spin
    }

    result = read8(devAddr7, 0x62); // read from RESULT_RANGE_VAL

    write8(devAddr7, 0x15, 0x07); // write 0x07 to SYSTEM_INTERRUPT_CLEAR

    return result;

}

int main(void) {

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USB_DEVICE_Init();
    MX_I2C1_Init();

    bringUpSensors();

    // initialize the USB buffer
    uint8_t usb_buf[5];
    int i;
    for (i=0; i<4; i++) {
        usb_buf[i] = 0;
    }
    usb_buf[4] = 10; // newline

    // main loop
    while (1) {

        // start range conversion on all four sensors
        write8(ADDR7_NORTH, 0x18, 0x01);
        write8(ADDR7_EAST, 0x18, 0x01);
        write8(ADDR7_SOUTH, 0x18, 0x01);
        write8(ADDR7_WEST, 0x18, 0x01);

        // now read from each one

        usb_buf[0] = readRangeContinuous(ADDR7_NORTH);
        if (usb_buf[0] < THRESH) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        }

        usb_buf[1] = readRangeContinuous(ADDR7_EAST);
        if (usb_buf[1] < THRESH) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        }

        usb_buf[2] = readRangeContinuous(ADDR7_SOUTH);
        if (usb_buf[2] < THRESH) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        }

        usb_buf[3] = readRangeContinuous(ADDR7_WEST);
        if (usb_buf[3] < THRESH) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        }

        CDC_Transmit_FS(&usb_buf, 5);

    }

}

void SystemClock_Config(void) {

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

static void MX_I2C1_Init(void) {

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x2000090E;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

}

static void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure port A pins : PA2, PA3, PA6
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure port B pins : PB0, PB4, PB8, PB10, PB12
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize all GPIO outputs low
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

}

void _Error_Handler(char *file, int line) {

    while(1) {

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_Delay(100);

    }

}

