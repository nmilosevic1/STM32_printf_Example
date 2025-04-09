/* my_board_info header
 *
 * Define all your board specific stuff here
 * **  LED related instructions are in the STM32_LED_Example project
 *
 * Check your board's user guide to determine your USART device and alter below definitions accordingly
 * e.g. section 6.9  USART communication
 *                   Pin name    Function      Virtual COM port
 *                   PD8         USART3 TX     SB5 ON and SB7 OFF
 *                   PD9         USART3 RX     SB6 ON and SB4 OFF
 * My pins are PD8 and PD9 (GPIOD port), device is USART3 (we will only use the TX pin here)
 *
 * Check your board's datasheet "block diagram" to determine which bus the USART device is connected to
 *   e.g. My USART3 device is connected to APB1 bus (update USART_BUS below to your USARTx's bus)

 * Check your board's datasheet "alternate function mapping" to determine which setting your USART TX pin requires
 *   e.g. Table 13. STM32F765xx, STM32F767xx, STM32F768Ax, and STM32F769xx alternate function mapping
 *     in the Port D section (because my pin is PD8)
 *     lists USART3_TX in the AF7 column, so my alternate function setting needs to be AF7 (update TX_AFT below to your AFT)
 *
 * Check your board's reference manual to determine which bit to set for the bus your USART device is connected to
 *   search for e.g. APB1ENR (or whichever bus your USART device is connected to)
 *     check the register definition for the bit you will need to set to enable clock for your USART
 *   e.g. Bit 18 USART3EN: USART3 clock enable (RCC_APB1ENR_USART3EN) update USART_EN_BIT below to your USART enable bit
 *
 * Check your reference manual to determine the default clock rate of the processor clock
 *   e.g. 5.2.2 HSI clock
 *       The HSI clock signal is generated from an internal 16 MHz RC oscillator and can be used directly as a system clock...
 *   (16 MHz is 16 million cycles per second, update CLOCK_FREQ below if your default is different)
 * - some board's specs aren't very specific so if your printf's aren't working and your settings in this file are otherwise all correct
 *   you can guess your correct clock rate by changing the delays in main.c to 2 seconds (easy to see/count)
 *     and then run the executable and check if the lights are blinking at the wrong rate.
 *     If they are blinking way too fast your CLOCK_FREQ is set too low.
 *
 * Check your reference manual to determine how to set the control bits
 * (I am using parity control DISabled, word length 8, 1 stop bit)
 *   e.g. 34.8.1 USART control register 1 (USART_CR1)
 *        Bit 10 PCE: Parity control enable (0 disabled, 1 enabled)
 *        Bit 12 (M0) +
 *          Bit 28 (M1): Word length where M[1:0] = 00 means 1 start bit, 8 data bits, n stop bits(n configured in CR2)
 *   e.g. 34.8.2 USART control register 2 (USART_CR2)
 *        Bits 13:12 STOP[1:0]: STOP bits (00 : 1 stop bit)
 */

#ifndef MY_BOARD_INFO_H_
#define MY_BOARD_INFO_H_

#include "stm32f7xx.h"       // board specific headers - replace with headers for your board
#include "stm32f767xx.h"

extern enum AFT AltFnType;

// LED stuff
#define NUM_LEDS          3                           // update to the number of user LEDs on your board
#define LED1              (1U << 0)  // pin0  GREEN   // update to the pins for your user LEDs
#define LED2              (1U << 7)  // pin7  BLUE
#define LED3              (1U << 14) // pin14 RED
#define LED1_PORT         GPIOB                       // update to the ports for your LEDs
#define LED2_PORT         GPIOB
#define LED3_PORT         GPIOB
#define GPIO_BUS          (*(volatile uint32_t *)(&(RCC->AHB1ENR)))// update "AHB1ENR" to your GPIO bus's enable register
#define GPIOAEN_BIT       RCC_AHB1ENR_GPIOAEN         // update to the GPIOAEN bit for your GPIO_BUS

// USART stuff
#define USART_VCP         USART3    // update to your board's Virtual COM Port (VCP) USART device
#define USART_TX_PIN      (1U << 8) // update to the USART_TX pin defined for your board's VCP
#define USART_TX_PORT     GPIOD     // update to the port for your USART_TX pin
#define USART_BUS         (*(volatile uint32_t *)(&(RCC->APB1ENR)))// update "APB1ENR" to your bus's enable register
#define USART_EN_BIT      RCC_APB1ENR_USART3EN  // update to your USART's enable bit
#define USART_ISR_TXE_POS USART_ISR_TXE  // update to your ISR_TXE transmit buffer empty const [likely (1 <<7)]

#define CLOCK_FREQ        (16 * 1000000)   // update for your board's clock frequency
#define BAUDRATE          115200    // update to the baud rate you want
#define TX_AFT            AF7       // update to your USART_TX pin alternate function type

#define USART_DATAWIDTH_8 0x0U      // 8 bits word length: 1 start bit, 8 data bits, n stop bits
#define USART_PARITY_NONE 0x0U      // parity control disabled
#define USART_STOPBIT_1   0x0U      // use 1 stop bit

#endif /* MY_BOARD_INFO_H_ */
