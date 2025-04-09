/* uart header */
#ifndef __UART_TX_H_
#define __UART_TX_H_

#include "my_board_info.h"

#include <stdint.h>

void uart_tx_init(USART_TypeDef *UARTx);
void uart_write(USART_TypeDef * USARTx, uint8_t byte);

#endif //__UART_H_
