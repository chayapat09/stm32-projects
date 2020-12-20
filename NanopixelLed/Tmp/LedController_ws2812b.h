#include <stdint.h>
#include "stm32f4xx_hal.h"
#define WS2812_FREQ							(800000)
#define TIMER_CLOCK_FREQ					(80000000)
#define TIMER_PERIOD						(TIMER_CLOCK_FREQ / WS2812_FREQ)
#define LED_NUMBER							(32)
#define LED_DATA_SIZE						(LED_NUMBER * 24)
#define RESET_SLOTS_BEGIN					(50)
#define RESET_SLOTS_END						(50)
#define WS2812_LAST_SLOT					(1)
#define LED_BUFFER_SIZE						(RESET_SLOTS_BEGIN + LED_DATA_SIZE + WS2812_LAST_SLOT + RESET_SLOTS_END)
#define WS2812_0							(TIMER_PERIOD / 3)
#define WS2812_1							(TIMER_PERIOD * 2 / 3)
#define WS2812_RESET						(0)

void setLedColor(uint8_t * ledBuffer , uint16_t ledNumber , uint8_t R , uint8_t G , uint8_t B);
