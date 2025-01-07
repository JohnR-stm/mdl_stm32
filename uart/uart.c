#include <stdint.h>

#ifdef STM32G0
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_usart.h"  
#endif /* STM32G0 */

#ifdef STM32F4
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#endif /* STM32F4 */

#ifdef STM32L4
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_usart.h"
#endif /* STM32L4 */

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
  uart_hw_config_gpio(&GpioUartInit);
  
  //-- config UART1--//
  UartInit_t UartInit;
    UartInit.uart_bus = CONF_UART1_BUS;
    UartInit.port = CONF_UART1_PORT;
    UartInit.uart_clk_source = CONF_UART1_CLK_SOURSE;
    UartInit.baud_rate = CONF_UART1_RATE;
    UartInit.data_width = CONF_UART1_DATA_WIDTH;
    UartInit.stop_bits = CONF_UART1_STOP_BITS;
    UartInit.parity = CONF_UART1_PARITY;
    UartInit.transfer_direction = CONF_UART1_DIRECTION;
  uart_hw_smple_config(&UartInit);
  
  //-- Interrupts Init --//
  usart_interrupts_init(CONF_UART1_PORT, uart_IRQ_vector, 5, 0);
}

//-----------------------------------------------------------------------------

void uart1_send_string(char *str)
{
  while (*str != '\0') {
    uart_hw_send_byte (*str++, CONF_UART1_PORT);
    while (!uart_hw_tx_complete (CONF_UART1_PORT)) 
    {
      // Wait for transmission to complete
    }
  }
}  

//-----------------------------------------------------------------------------

void uart1_send_buf(uint8_t Buf[], uint16_t sz)
{
  uint16_t cnt = 0;
  while (cnt < sz)
  {
    uart_hw_send_byte (Buf[cnt], CONF_UART1_PORT);
    while (!uart_hw_tx_complete (CONF_UART1_PORT));
    cnt++;
  }
}  

//-----------------------------------------------------------------------------

uint8_t uart1_read_byte(void)
{
  uint8_t val;
  uart_hw_receive_byte (&val, CONF_UART1_PORT);
  return val;
}


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void uart1_IRQ_handler(void) // USART2_IRQHandler(void)
{
  if(uart_hw_IsData(CONF_UART1_PORT))
  {
    uart1_handler(); // Extern function
  }
  else
    uart_hw_clrORE_IDLE_EOB(CONF_UART1_PORT);
}