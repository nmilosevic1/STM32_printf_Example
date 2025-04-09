/* gpio implementation */
#include "gpio.h"

/* Initialize the user LEDs
 * Enable the clock access for the ports the LEDs are connected to
 * configure all led pins as OUTPUT pins (0b 01)
 */
void gpio_leds_init() {
  for (uint8_t led_idx=0; led_idx<NUM_LEDS; ++led_idx) {
    SET_BIT(GPIO_BUS, (1U << gpio_get_EN_bit(myLeds[led_idx].GPIOx)));
    gpio_set_pin_mode(myLeds[led_idx].GPIOx, myLeds[led_idx].pin, OUTPUT);
  }
}

/* Turn on or off all user LEDs
 * @param on - true to turn on, false to turn off
 */
void gpio_all_leds(bool on) {
  if (on) {
    for (uint8_t led_idx=0; led_idx<NUM_LEDS; ++led_idx) {
      myLeds[led_idx].GPIOx->ODR |= myLeds[led_idx].pin;
    }
  } else {
    for (uint8_t led_idx=0; led_idx<NUM_LEDS; ++led_idx) {
      myLeds[led_idx].GPIOx->ODR &=~ myLeds[led_idx].pin;
    }
  }
}

/* Toggle all user LEDs
 */
void gpio_toggle_all_leds() {
  for (uint8_t led_idx=0; led_idx<NUM_LEDS; ++led_idx) {
    myLeds[led_idx].GPIOx->ODR ^= myLeds[led_idx].pin;
  }
}

/* Toggle the given user LED
 * @param GPIOx the port the pin is located in
 * @param pin - the pin of the LED to toggle
 */
void gpio_toggle_one_led(GPIO_TypeDef *GPIOx, uint32_t pin) {
  GPIOx->ODR ^= pin;
}

/* Turn on or off one user LED
 * @param GPIOx the port the pin is located in
 * @param pin - the pin of the LED to toggle
 * @param on - true to turn on, false to turn off
 */
void gpio_one_led(GPIO_TypeDef *GPIOx, uint32_t pin, bool on) {
  if (on) {
    GPIOx->ODR |= pin;
  } else {
    GPIOx->ODR &=~ pin;
  }
}

/* Function to set the pin mode reg (MODER) of a particular pin in a particular port
 * @param GPIOx the port the pin is located in
 * @param pin the pin value of the pin to set e.g. (1 << 4) equates to pin 4
 * @param mode : 00 is INPUT, 01 is OUTPUT, 10 is ALTERNATE, 11 is ANALOG
 */
void gpio_set_pin_mode(GPIO_TypeDef *GPIOx, uint32_t pin, uint32_t mode) {
  // POSITION_VAL macro counts the bits to the right of the set bit, telling us which pin it actually is
  uint32_t pinPos = POSITION_VAL(pin);          // the actual pin position
  uint32_t modeRegLoc = pinPos * 2U;            // the MODER reg has 2 bits per pin
  uint32_t modePinsToClear = 0x3 << modeRegLoc; // 0x3 is 2 1's. shift the 2 ones into the position of the mode bits for this pin
  uint32_t modePinsToSet = mode << modeRegLoc;  // shift the 2 mode bits into the position of the mode bits for this pin

  // Clear the relevant bits in the MODER REG and Set the bits to mode param
  MODIFY_REG(GPIOx->MODER, modePinsToClear, modePinsToSet);
}

/* Function to set the alternate function pin mode reg (AFR H/L) of a particular pin in a particular port
 * @param GPIOx the port the pin is located in
 * @param pin the pin value of the pin to set e.g. (1 << 4) equates to pin 4
 * @param mode : 0000 is AF0, 0001 is AF1... 1111 is AF15
 */
void gpio_set_alternate_function_mode(GPIO_TypeDef *GPIOx, uint32_t pin, uint32_t mode) {
  uint32_t pinPos = POSITION_VAL(pin);          // the actual pin position

  // AFRLow REG is for pins 0-7, AFRHigh REG is for pins 8-15
  uint32_t i = 0; // for AFRL
  if (pinPos > 7) {
    i = 1;  // use AFRH
  }

  uint32_t modeRegLoc = (pinPos - (i*8U)) * 4U; // the AFR reg has 4 bits per pin, account for AFRH
  uint32_t modePinsToClear = 0xf << modeRegLoc; // 0xf is 4 1's. shift the 4 ones into the position of the mode bits for this pin
  uint32_t modePinsToSet = mode << modeRegLoc;  // shift the 4 mode bits into the position of the mode bits for this pin

  // Clear the relevant bits in the AFR REG and Set the bits to mode param
  MODIFY_REG(GPIOx->AFR[i], modePinsToClear, modePinsToSet);
}

/* Get the correct bit for the given GPIO in the bus enable register
 * @param GPIOx - the GPIO port
 * @return the GPIOxEN bit in the enable register GPIO_BUS (e.g. for stm32f767 GPIOAEN is bit 0 .... GPIOKEN is bit 10)
 */
uint32_t gpio_get_EN_bit(GPIO_TypeDef *GPIOx) {
  const uint64_t GPIO_SIZE = (GPIOB_BASE - GPIOA_BASE);           // Size of a GPIO port
  uint64_t locFromGpioA = (uint32_t)GPIOx - (uint32_t)GPIOA_BASE; // mem loc of given gpio device offset from base of GPIOA
  uint32_t gpioDevNum = locFromGpioA / GPIO_SIZE;                 // the number of the gpio port (e.g. GPIOC would be 2 here)
  return gpioDevNum + POSITION_VAL(GPIOAEN_BIT);                  // add on the position of GPIOAEN in case it isn't zero
}
