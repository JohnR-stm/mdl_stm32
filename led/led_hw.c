#include <stdint.h>

#ifdef STM32G0
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"
#endif /* STM32G0 */

#ifdef STM32F3
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_gpio.h"
#endif /* STM32F3 */

#include "led_config.h"

#include "led_hw.h"

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void led_init(void)
{
  //--- GPIO Ports Clock Enable ---//
  if  ( LED_CLK_IsEnabled(LED_GREEN_BUS) )
  { } else LED_CLK_Enable(LED_GREEN_BUS);
  //--- GREEN LED Init ---//
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_GPIO_ResetOutputPin(LED_GREEN_PORT, LED_GREEN);
  GPIO_InitStruct.Pin = LED_GREEN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GREEN_PORT, &GPIO_InitStruct);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void led_green_on(void)
{
  LL_GPIO_SetOutputPin(LED_GREEN_PORT, LED_GREEN);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

void led_green_off(void)
{
  LL_GPIO_ResetOutputPin(LED_GREEN_PORT, LED_GREEN);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------


