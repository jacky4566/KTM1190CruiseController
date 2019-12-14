#include "stm32l4_dma.h"
#include "stm32l4_timer.h"

stm32l4_dma_t stm32l4_DAC_DMA;
stm32l4_timer_t stm32l4_DAC_timer;

static int DACresolution = 12;

uint32_t desiredValue;

uint32_t DAC_DMA_OPTION = DMA_OPTION_MEMORY_TO_PERIPHERAL |
                          DMA_OPTION_CIRCULAR |
                          DMA_OPTION_PERIPHERAL_DATA_SIZE_32 |
                          DMA_OPTION_MEMORY_DATA_SIZE_32 |
                          DMA_OPTION_PRIORITY_MEDIUM ;

void setup() {
  Serial.begin(9600);
  analogWriteResolution(DACresolution);
  analogWrite(PIN_DAC0, 0); //use board manager to setup dac
  analogWrite(PIN_DAC1, 0); //use board manager to setup dac

  //Setup timer 7 to trigger arbitraty 1MHz (@80MHZ)
  stm32l4_timer_create(&stm32l4_DAC_timer, TIMER_INSTANCE_TIM7, NULL, 0);
  stm32l4_timer_enable(&stm32l4_DAC_timer, (stm32l4_timer_clock(&stm32l4_DAC_timer) / 4000000) - 1, 4 - 1, TIMER_OPTION_COUNT_PRELOAD, NULL, NULL, 0);
  
  //Setup DMA to transfer into DAC
  stm32l4_dma_create(&stm32l4_DAC_DMA, DMA_CHANNEL_DMA1_CH3_DAC1, DMA_OPTION_PRIORITY_MEDIUM);
  stm32l4_dma_enable(&stm32l4_DAC_DMA, NULL, NULL);
  stm32l4_dma_start(&stm32l4_DAC_DMA, (uint32_t)&DAC->DHR12R1, (uint32_t)desiredValue, 1, DAC_DMA_OPTION);

  //enable DAC DMA, Trigger on TIM7, Timer trigger EN
  DAC->CR = (uint32_t) DAC->CR | 0x1014;

  //start timer
  stm32l4_timer_start(&stm32l4_DAC_timer, false);
}

void loop() {
  desiredValue = desiredValue + 64;
  DAC->DHR12R2 = desiredValue; //Verify 
  delay(100); //keep things slow
}
