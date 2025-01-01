#ifndef _MDL_UART_HW_H_
#define _MDL_UART_HW_H_


#define USART_PORT USART2


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

  
  
void uart_hw_configure_gpio(UartGpioInit_t * GpioUartInit);
void uart_hw_configure(void);

void usart_send_byte (uint8_t value);
uint8_t usart_tx_complete (void);
void usart_receive_byte (uint8_t * value);



#endif /* _MDL_UART_HW_H_ */