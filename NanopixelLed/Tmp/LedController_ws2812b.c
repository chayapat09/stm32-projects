#include "LedController_ws2812b.h"

static uint8_t LEDbuffer[LED_BUFFER_SIZE];

void setLedColor(uint8_t * ledBuffer , uint16_t ledNumber , uint8_t R , uint8_t G , uint8_t B) {
	uint8_t binaryBuffer[24];
	ledNumber = ledNumber % LED_NUMBER;
	for (uint8_t i = 7 ; i >= 0 ; i--) {
		binaryBuffer[i] 	= ((G >> i) & 1) ? WS2812_1 : WS2812_0;
		binaryBuffer[i+8] 	= ((R >> i) & 1) ? WS2812_1 : WS2812_0;
		binaryBuffer[i+16] 	= ((B >> i) & 1) ? WS2812_1 : WS2812_0;
	}

	for (uint8_t i = 0 ; i < 24 ; i++) {
		ledBuffer[RESET_SLOTS_BEGIN + ledNumber * 24 + i] = binaryBuffer[i];
	}


}

// Set Brightness
// Set color
// Update / pin and buffer and ... requried
