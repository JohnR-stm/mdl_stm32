#include <stdint.h>

#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_usart.h"

#include "uart_hw.h"
#include "uart_config.h"
#include "uart.h"


void uart_init_all(void)
{
  // UART 1 //
  //-- config GPIO and GPIO_PORT--//
  UartGpioInit_t GpioUartInit = {0};
    GpioUartInit.port_tx_bus = CONF_UART1_GPIO_TX_BUS;
    GpioUartInit.port_rx_bus = CONF_UART1_GPIO_RX_BUS;
    GpioUartInit.port_tx = CONF_UART1_GPIO_TX_PORT;
    GpioUartInit.port_rx = CONF_UART1_GPIO_RX_PORT;
    GpioUartInit.AF_tx = CONF_UART1_GPIO_TX_AF;
    GpioUartInit.AF_rx = CONF_UART1_GPIO_RX_AF;
    GpioUartInit.pin_tx = CONF_UART1_GPIO_TX_PIN;  //LL_GPIO_PIN_2
    GpioUartInit.pin_rx = CONF_UART1_GPIO_RX_PIN; //LL_GPIO_PIN_3
  uart_hw_configure_gpio(&GpioUartInit);
  
  //-- config UART--//
  uart_hw_configure();
  
}

  
void usart_send_string(char *str)
{
  while (*str != '\0') {
    usart_send_byte (*str++);
    while (!usart_tx_complete ()) 
    {
      // Wait for transmission to complete
    }
  }
}  