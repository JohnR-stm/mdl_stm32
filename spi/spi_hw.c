#include <stdint.h>

#ifdef STM32G0
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_spi.h"  
#include "stm32g0xx_ll_dma.h"
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


#include "spi_hw.h"

//-----------------------------------------------------------------------------

static void spi_pins_init(void);
static void spi_port_init(void);
static void spi_dma_init(void);
static void spi_dma_interrupts_enable(void);

void set_SPI_addr (void);

void set_SPI_buf_len (uint32_t data_len);

void set_SPI_buf_addr (uint32_t * addr);



//-----------------------------------------------------------------------------

void spi_init_all(void)
{
  spi_pins_init();
  spi_port_init();
  spi_dma_init();
  spi_dma_interrupts_enable();
}

//-----------------------------------------------------------------------------

static void spi_pins_init(void)
{
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  
  //--- SPI2 GPIO Configuration ---//
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  //---- PA0 --> SPI2_SCK  ----//
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
  
  //---- PC2 --> SPI2_MISO ----//
  //---- PC3 --> SPI2_MOSI ----//
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct); 
}

//------------------------------------------------------------------------------
// Spi2 slave mode RX Only
//------------------------------------------------------------------------------

static void spi_port_init(void)
{
  //--- SPI 2 INIT ---//
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
  
  //--- SPI2  configuration ---//
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  SPI_InitStruct.TransferDirection = LL_SPI_SIMPLEX_RX;         // SPIx->CR1 -- RX Only
  SPI_InitStruct.Mode = LL_SPI_MODE_SLAVE;                      // Slave no (MSTR | SSI)
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;            // Data - 16 bit (CR2 DS)
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;                         // SS not used
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;      //  8 Mhz
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;// NO CRC
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);           // CR2, SPI_CR2_FRF
  //LL_SPI_EnableNSSPulseMgt(SPI2);                               // CR2, SPI_CR2_NSSP (Only in Master Mode)
  
  //--- DMA RX Enable ---//
  //LL_SPI_EnableDMAReq_TX(SPI2);
  LL_SPI_EnableDMAReq_RX(SPI2);
  //--- SPI 2 Enable ---//
  LL_SPI_Enable(SPI2);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------

static void spi_dma_init(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  //LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) Buf_ADC);
  //LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, (uint32_t) (CCD - 2));
    
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);   // From SPI to Buf
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_MEDIUM);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);                                 // Circular
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
  
  //DMA1_Channel1->CCR = 0;
  
  // | DMA_CCR_TEIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0  | DMA_CCR_MSIZE_0);
  //DMA1_Channel1->CCR |= (DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN);  
  
  //DMAMUX
  // SPI2_RX -- DMAMUX input = 18
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_SPI2_RX);
  
  
}

//------------------------------------------------------------------------------

static void spi_dma_interrupts_enable(void)
{
  DMA1_Channel1->CCR |= DMA_CCR_TCIE; // Transfer Complete interrupt enable 
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0)); 
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

//------------------------------------------------------------------------------
// DMA addreses init
//------------------------------------------------------------------------------

inline void set_SPI_addr (void)
{
  uint32_t spi_addr;
  spi_addr = LL_SPI_DMA_GetRegAddr(SPI2);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, spi_addr);
}

//------------------------------------------------------------------------------

inline void set_SPI_buf_len (uint32_t data_len)
{
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, data_len);
}

//------------------------------------------------------------------------------

inline void set_SPI_buf_addr (uint32_t * addr)
{
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) addr);
}

//------------------------------------------------------------------------------
// DMA Buffer 

void spi_dma_buf (uint32_t * addr, uint32_t data_len)
{
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  
  LL_SPI_EnableDMAReq_RX(SPI2);
  
  set_SPI_addr ();
  set_SPI_buf_len (data_len);
  set_SPI_buf_addr (addr);
  
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}





//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// INTERRUPT
//------------------------------------------------------------------------------

void DMA1_Channel1_IRQHandler(void) // in spi.c, in config
{
  if(LL_DMA_IsActiveFlag_TC1(DMA1))
  {
    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_HT1(DMA1);
    LL_DMA_ClearFlag_TE1(DMA1);
    //spi_handler(); // Extern function
  }
  else
  {
    // clear flags
    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_ClearFlag_HT1(DMA1);
    LL_DMA_ClearFlag_TE1(DMA1);  // Transfer error
  }   
  spi_handler(); // Extern function
 
}


//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
