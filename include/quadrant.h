#include "stm32f0xx_hal.h"
#include "usb_device.h"

// values from the datasheet
#define VL_ADDR7_DEFAULT 0x29
#define VL_MODEL_ID 0xb4

// U3 on EN1
#define NORTH_ADDR7 0x2a
#define NORTH_EN_PORT GPIOA
#define NORTH_EN_PIN GPIO_PIN_0
#define NORTH_LED_PORT GPIOC
#define NORTH_LED_PIN GPIO_PIN_13

// U5 on EN2
#define EAST_ADDR7 0x2b
#define EAST_EN_PORT GPIOA
#define EAST_EN_PIN GPIO_PIN_15
#define EAST_LED_PORT GPIOB
#define EAST_LED_PIN GPIO_PIN_4

// U4 on EN3
#define SOUTH_ADDR7 0x2c
#define SOUTH_EN_PORT GPIOA
#define SOUTH_EN_PIN GPIO_PIN_8
#define SOUTH_LED_PORT GPIOB
#define SOUTH_LED_PIN GPIO_PIN_15

// U1 on EN4
#define WEST_ADDR7 0x2d
#define WEST_EN_PORT GPIOA
#define WEST_EN_PIN GPIO_PIN_7
#define WEST_LED_PORT GPIOB
#define WEST_LED_PIN GPIO_PIN_0

#define LED_THRESH 180

