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
//
//-----------------------------------------------------------------------------

void uart_hw_configure_gpio(UartGpioInit_t * GpioUartInit) {
  
  // bus tx
  if(CONF_UART_GPIO_IsEnabledClock( GpioUartInit->port_tx_bus ))
  {} else
    CONF_UART_GPIO_EnabledClock( GpioUartInit->port_tx_bus );
  // bus rx
  if(CONF_UART_GPIO_IsEnabledClock( GpioUartInit->port_rx_bus))
  {} else
    CONF_UART_GPIO_EnabledClock( GpioUartInit->port_rx_bus );
  
  
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
  

  

 
  
  //USART2 GPIO Configuration
  //PA2   ------> USART2_TX
  //PA3   ------> USART2_RX
  /*
  //TX
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //RX
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  */


}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void uart_hw_configure(void) {
  
  
    LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

  // Peripheral clock enable 
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

   


  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART2);
  LL_USART_ConfigAsyncMode(USART2);



  LL_USART_Enable(USART2);

  // Polling USART2 initialisation 
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }


  
  
  
  
  /*
  LL_USART_InitTypeDef USART_InitStruct = {0};
  
  LL_APB1_GRP1_EnableClock(USART_APB);
  LL_RCC_SetUSARTClockSource(USART_Clk); 
  
  USART_InitStruct.BaudRate = USART_RATE;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART_PORT, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART_PORT);
  LL_USART_Enable(USART_PORT); 
*/
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

void usart_send_byte (uint8_t value)
{
   LL_USART_TransmitData8(USART_PORT, value);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void usart_receive_byte (uint8_t * value)
{
   *value = LL_USART_ReceiveData8(USART_PORT);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

uint8_t usart_tx_complete (void)
{
  return LL_USART_IsActiveFlag_TXE(USART_PORT) == 1 ? 1 : 0;
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
