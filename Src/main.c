/* main.c
 * Simple printf example for STM32
 *
 * See README for initial setup
 * Update my_board_info.h to meet the specification for your board (if not stm32f767)
 *
 */
#include "my_board_info.h"

#include <stdio.h>
#include <stdint.h>
#include "gpio.h"
#include "systick.h"
#include "uart.h"

struct LedDef myLeds[NUM_LEDS] = {
    {LED1_PORT, LED1}, // update for your board's LED(s) if you don't have 3
    {LED2_PORT, LED2},
    {LED3_PORT, LED3}
};

int main(void) {
  gpio_leds_init();        // initialize the LEDs
  gpio_all_leds(ON);       // start with all LEDs lit

  uart_tx_init(USART_VCP); // initialize the UART
  printf("Hello STM32!\n");

  uint8_t counter = 0; // (size 1 byte, rolls over at 0xFF)
  while (1) {  // loop forever
    systick_delay_msecs(1200);
    gpio_all_leds(OFF); // turn off all LEDs
    printf("[0x%02X] Toggle on each LED...\n", ++counter);

    // turn on each light one by one until all are on
    for (uint32_t led_idx=0; led_idx<NUM_LEDS; ++led_idx) {
      systick_delay_msecs(200);
      gpio_toggle_one_led(myLeds[led_idx].GPIOx, myLeds[led_idx].pin);
    }
  }
}
