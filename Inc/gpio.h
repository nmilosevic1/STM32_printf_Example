/* gpio header file */
#ifndef __GPIO_H_
#define __GPIO_H_

#include "my_board_info.h"

#include <stdint.h>
#include <stdbool.h>

#define ON  true
#define OFF false

void gpio_leds_init();
void gpio_all_leds(bool on);
void gpio_one_led(GPIO_TypeDef *GPIOx, uint32_t pin, bool on);
void gpio_toggle_all_leds();
void gpio_toggle_one_led(GPIO_TypeDef *GPIOx, uint32_t pin);
void gpio_set_pin_mode(GPIO_TypeDef *GPIOx, uint32_t pin, uint32_t mode);
void gpio_set_alternate_function_mode(GPIO_TypeDef *GPIOx, uint32_t pin, uint32_t mode);
uint32_t gpio_get_EN_bit(GPIO_TypeDef *GPIOx);

// I/O mode values (GPIOx_MODER)
enum {
  INPUT = 0x0, // 0b 00
  OUTPUT,      // 0b 01
  ALTERNATE,   // 0b 10
  ANALOG       // 0b 11
};

// Alternate Function values (GPIOx_AFRL and GPIOx_AFRH)
enum AFT {
  AF0 = 0x0, // 0b 0000
  AF1,       // 0b 0001
  AF2,       // 0b 0010
  AF3,       // 0b 0011
  AF4,       // 0b 0100
  AF5,       // 0b 0101
  AF6,       // 0b 0110
  AF7,       // 0b 0111
  AF8,       // 0b 1000
  AF9,       // 0b 1001
  AF10,      // 0b 1010
  AF11,      // 0b 1011
  AF12,      // 0b 1100
  AF13,      // 0b 1101
  AF14,      // 0b 1110
  AF15       // 0b 1111
};

struct LedDef {
  GPIO_TypeDef *GPIOx;
  uint32_t pin;
};

extern struct LedDef myLeds[NUM_LEDS];

#endif// __GPIO_H_
