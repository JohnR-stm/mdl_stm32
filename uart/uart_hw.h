#ifndef _MDL_UART_HW_H_
#define _MDL_UART_HW_H_

//----------------------------------------------------------------------------

typedef struct{
  
  uint32_t port_tx_bus;
  uint32_t pin_tx;
  GPIO_TypeDef * port_tx;
  uint32_t AF_tx;
  
  uint32_t port_rx_bus;
  uint32_t pin_rx;
  GPIO_TypeDef * port_rx;
  uint32_t AF_rx;

} UartGpioInit_t;

//----------------------------------------------------------------------------

typedef struct{
  
  uint32_t uart_bus;
  uint32_t uart_clk_source;
  USART_TypeDef * port;
  uint32_t prescaler;
  uint32_t baud_rate;
  uint32_t data_width;
  uint32_t stop_bits;
  uint32_t parity;
  uint32_t transfer_direction;
  uint32_t hardware_flow_cntrl;
  uint32_t oversampling;
  
} UartInit_t;
  
//----------------------------------------------------------------------------
  

void uart_hw_config_gpio(UartGpioInit_t * GpioUartInit);
void uart_hw_config(UartInit_t * UartInit);
void uart_hw_smple_config(UartInit_t * UartInit);

void usart_interrupts_init (USART_TypeDef * port, IRQn_Type irq_usart, uint32_t usart_prio, uint32_t usart_subprio);
void usart_interrupts_deinit (USART_TypeDef * port, IRQn_Type irq_usart);

void uart_hw_send_byte (uint8_t value, USART_TypeDef *USARTx);
void uart_hw_receive_byte (uint8_t * value, const USART_TypeDef *USARTx);
uint8_t uart_hw_tx_complete (const USART_TypeDef *USARTx);

//-----------------------------------------------------------------------------

__STATIC_INLINE uint32_t uart_hw_IsData(const USART_TypeDef *USARTx)
{
  return (LL_USART_IsActiveFlag_RXNE(USARTx) && LL_USART_IsEnabledIT_RXNE(USARTx));
  //return ((READ_BIT(USARTx->ISR, USART_ISR_RXNE_RXFNE) == (USART_ISR_RXNE_RXFNE)) ? 1UL : 0UL);
}

//-----------------------------------------------------------------------------

__STATIC_INLINE void uart_hw_clrORE_IDLE_EOB(USART_TypeDef *USARTx)
{
  WRITE_REG(USARTx->ICR, USART_ICR_ORECF);
  WRITE_REG(USARTx->ICR, USART_ICR_ORECF);
  WRITE_REG(USARTx->ICR, USART_ICR_EOBCF);
}

//----------------------------------------------------------------------------

#endif /* _MDL_UART_HW_H_ */