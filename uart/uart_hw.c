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

#include "uart_config.h"

#include "uart_hw.h"

//-----------------------------------------------------------------------------
// universal function uart pins init
//-----------------------------------------------------------------------------

void uart_hw_config_gpio(UartGpioInit_t * GpioUartInit) 
{
  // bus tx
  if(CONF_UART_GPIO_IsEnabledClock( GpioUartInit->port_tx_bus ))
  {} else
    CONF_UART_GPIO_EnableClock( GpioUartInit->port_tx_bus );
  // bus rx
  if(CONF_UART_GPIO_IsEnabledClock( GpioUartInit->port_rx_bus))
  {} else
    CONF_UART_GPIO_EnableClock( GpioUartInit->port_rx_bus );
  
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  //TX
  GPIO_InitStruct.Pin = GpioUartInit->pin_tx;             // Pin TX
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = GpioUartInit->AF_tx;        // AFIO
  LL_GPIO_Init(GpioUartInit->port_tx, &GPIO_InitStruct);  // PORT TX
  //RX
  GPIO_InitStruct.Pin = GpioUartInit->pin_rx;             // Pin RX
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = GpioUartInit->AF_rx;        // AFIO
  LL_GPIO_Init(GpioUartInit->port_rx, &GPIO_InitStruct);  // PORT RX
}

//-----------------------------------------------------------------------------
// universal function uart init
//-----------------------------------------------------------------------------

void uart_hw_config(UartInit_t * UartInit) 
{
  //uart init bus clock 
  if( CONF_UART_IsEnabledClock(UartInit->uart_bus) )
  {} else
    CONF_UART_EnableClock(UartInit->uart_bus);    //CONF_UART1_BUS
  //uart clock source
  CONF_UART_ClockSource(UartInit->uart_clk_source);  //LL_RCC_USART2_CLKSOURCE_PCLK1

    
  LL_USART_InitTypeDef USART_InitStruct = {0};
  USART_InitStruct.PrescalerValue = UartInit->prescaler;
  USART_InitStruct.BaudRate = UartInit->baud_rate;
  USART_InitStruct.DataWidth = UartInit->data_width;
  USART_InitStruct.StopBits = UartInit->stop_bits;
  USART_InitStruct.Parity = UartInit->parity;
  USART_InitStruct.TransferDirection = UartInit->transfer_direction;
  USART_InitStruct.HardwareFlowControl = UartInit->hardware_flow_cntrl;
  USART_InitStruct.OverSampling = UartInit->oversampling;
  LL_USART_Init(UartInit->port, &USART_InitStruct);

  //LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  //LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(UartInit->port);
  LL_USART_ConfigAsyncMode(UartInit->port);
  LL_USART_Enable(UartInit->port);

  // Polling USART2 initialisation 
  while((!(LL_USART_IsActiveFlag_TEACK(UartInit->port))) || (!(LL_USART_IsActiveFlag_REACK(UartInit->port))))
  { }
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void uart_hw_smple_config(UartInit_t * UartInit) 
{
  UartInit->prescaler = LL_USART_PRESCALER_DIV1;
  UartInit->hardware_flow_cntrl = LL_USART_HWCONTROL_NONE;
  UartInit->oversampling = LL_USART_OVERSAMPLING_16;

  uart_hw_config(UartInit);
}


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
/*
void usart_interrupts_init (void){
    
  LL_USART_EnableIT_RXNE(USART_PORT);   /// Enable interrupts --- P.1240 RM 
  //LL_USART_EnableIT_ERROR(USART_PORT);  /// ERR Interrupts Enable (CR3)
  NVIC_SetPriority(USART_IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),USART_PRIO, USART_SUBPRIO)); 
  NVIC_EnableIRQ(USART_IRQ);
}


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void usart_interrupts_deinit (void){
    
  LL_USART_DisableIT_RXNE(USART_PORT);
  NVIC_DisableIRQ(USART_IRQ);
}
*/
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void uart_hw_send_byte (uint8_t value, USART_TypeDef *USARTx)
{
   LL_USART_TransmitData8(USARTx, value);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void uart_hw_receive_byte (uint8_t * value, const USART_TypeDef *USARTx)
{
   *value = LL_USART_ReceiveData8(USARTx);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

uint8_t uart_hw_tx_complete (const USART_TypeDef *USARTx)
{
  return LL_USART_IsActiveFlag_TXE(USARTx) == 1 ? 1 : 0;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
/*
void USART2_IRQHandler(void)
{
  if(LL_USART_IsActiveFlag_RXNE(USART_PORT) && LL_USART_IsEnabledIT_RXNE(USART_PORT)) /// Receive data register not empty P.1236 RM
  {
    uart_interrupt_handler(); 
  }
  else
  {
    //-- clear error flags --//
    LL_USART_ClearFlag_ORE(USART_PORT);
    LL_USART_ClearFlag_IDLE(USART_PORT);
    LL_USART_ClearFlag_EOB(USART_PORT);
    //uint8_t tmp;
    //usart_receive_byte (&tmp);
  }
}
*/

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
