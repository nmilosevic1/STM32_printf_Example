/* uart implentation */
#include "uart.h"
#include "gpio.h"

static uint16_t compute_uart_divider(uint32_t clockFreq, uint32_t baudrate);
static void set_baudrate(USART_TypeDef * USARTx, uint32_t clockFreq, uint32_t baudrate);

// override putchar or fputc to trick printf into sending chars to the uart
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

// send the chars from printf to the USART device instead
PUTCHAR_PROTOTYPE {
  uart_write(USART_VCP, ch);
  return ch;
}

/* Initialize and enable the USART module
 * @param USARTx - the USART device
 */
void uart_tx_init(USART_TypeDef *USARTx) {
  // Enable clock access for TX pin
  uint32_t tx_EN_bit_pos = (1U << gpio_get_EN_bit(USART_TX_PORT));
  SET_BIT(GPIO_BUS, tx_EN_bit_pos);

  // Set pin mode to ALTERNATE
  gpio_set_pin_mode(USART_TX_PORT, USART_TX_PIN, ALTERNATE);

  // Set alternate function for USART TX pin
  gpio_set_alternate_function_mode(USART_TX_PORT, USART_TX_PIN, TX_AFT);

  SET_BIT(USART_BUS, USART_EN_BIT); // Enable clock access to USART module

  // set parity control, data width and stop bits
  USARTx->CR1 &=~ USART_CR1_UE;  // clear UART ENABLE while we configure it
  MODIFY_REG(USARTx->CR1, (USART_CR1_PS | USART_CR1_PCE | USART_CR1_M), (USART_PARITY_NONE | USART_DATAWIDTH_8));
  MODIFY_REG(USARTx->CR2, USART_CR2_STOP, USART_STOPBIT_1);

  // enable transmit (from board to PC)
  MODIFY_REG(USARTx->CR1, USART_CR1_TE, USART_CR1_TE);

  set_baudrate(USARTx, CLOCK_FREQ, BAUDRATE); // set the baud rate

  USARTx->CR1 |= USART_CR1_UE;  // set UART ENABLE
}

/* Function to write to the transmit data REG
 * @param USARTx - the USART device
 * @byte the byte to write
 */
void uart_write(USART_TypeDef * USARTx, uint8_t byte) {
  while (!(USARTx->ISR & USART_ISR_TXE_POS)) {} // wait until transmit data register is empty
  USARTx->TDR = byte;  // write byte into transmit data register
}

static uint16_t compute_uart_divider(uint32_t clockFreq, uint32_t baudrate) {
  return (clockFreq + (baudrate / 2U)) / baudrate;
}

static void set_baudrate(USART_TypeDef * USARTx, uint32_t clockFreq, uint32_t baudrate) {
  USARTx->BRR = compute_uart_divider(clockFreq, baudrate);
}
