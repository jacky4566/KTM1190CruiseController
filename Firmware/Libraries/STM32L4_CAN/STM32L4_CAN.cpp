
/**
 * Function implementations for enabling and using CAN messaging.
 */
#include "STM32L4_CAN.h"

STM32L4_CAN::STM32L4_CAN(enum BITRATE bitrate){
	RCC->APB1ENR1 |= 0x2000000UL;  	// Enable CAN clock 
	RCC->APB2ENR |= 0x1UL;			// Enable AFIO clock
	GPIOB-> AFR[1] &= ~(0xFFUL);	// Clear alternate functions PB8 and PB9
	GPIOB-> AFR[1] |= 0x9UL;		// Set PB8 alternate function 9, CAN1_RX
	GPIOB-> AFR[1] |= 0x90UL;		// Set PB9 alternate function 9, CAN1_TX
 
	RCC->APB2ENR |= 0x8UL;			// Enable GPIOB clock
	GPIOB->OTYPER &= ~(0x300UL);	//Clear types, Output push-pull (reset state)
	GPIOB->MODER  &= ~(0xF0000UL);	//Clear modes
	GPIOB->MODER  |= 0x80000UL;		//PB9, Alternate Function
	GPIOB->ODR |= 0x1UL << 8;		//pulldown PB8
  
	CAN1->MCR = 0x51UL;			    // Set CAN to initialization mode

	// Set bit rates 
	CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF)); 
	CAN1->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);
	
	//CAN1->BTR |= 0x40000000UL; //loop back mode
 
	// Configure Filters to default values
	CAN1->FM1R |= 0x1C << 8;              // Assign all filters to CAN1
	CAN1->FMR  |=   0x1UL;                // Set to filter initialization mode
	CAN1->FA1R &= ~(0x1UL);               // Deactivate filter 0
	CAN1->FS1R |=   0x1UL;                // Set first filter to single 32 bit configuration
 
	CAN1->sFilterRegister[0].FR1 = 0x0UL; // Set filter registers to 0
	CAN1->sFilterRegister[0].FR2 = 0x0UL; // Set filter registers to 0
	CAN1->FM1R &= ~(0x1UL);               // Set filter to mask mode
 
	CAN1->FFA1R &= ~(0x1UL);			  // Apply filter to FIFO 0  
	CAN1->FA1R  |=   0x1UL;               // Activate filter 0
	
	CAN1->FMR   &= ~(0x1UL);			  // Deactivate initialization mode
	CAN1->MCR   &= ~(0x1UL);              // Set CAN to normal mode 
	
	//while (CAN1->MSR & 0x1UL); //wait for sync
}

uint8_t STM32L4_CAN::CANAddFilter(uint16_t id){
	 
	 if (filterIDpointer == 112){
		 return 1;
	 }
	 
	 CAN1->FMR  |=   0x1UL;                // Set to filter initialization mode
	 
	 switch(filterIDpointer%4){
		 case 0:
				// if we need another filter bank, initialize it
				CAN1->FA1R |= 0x1UL <<(filterIDpointer/4);
				CAN1->FM1R |= 0x1UL << (filterIDpointer/4);
		    CAN1->FS1R &= ~(0x1UL << (filterIDpointer/4)); 
				
				CAN1->sFilterRegister[filterIDpointer/4].FR1 = (id << 5) | (id << 21);
		    CAN1->sFilterRegister[filterIDpointer/4].FR2 = (id << 5) | (id << 21);
				break;
		 case 1:
			  CAN1->sFilterRegister[filterIDpointer/4].FR1 &= 0x0000FFFF;
				CAN1->sFilterRegister[filterIDpointer/4].FR1 |= id << 21;
			  break;
		 case 2:
				CAN1->sFilterRegister[filterIDpointer/4].FR2 = (id << 5) | (id << 21);
		    break;
		 case 3:
			  CAN1->sFilterRegister[filterIDpointer/4].FR2 &= 0x0000FFFF;
				CAN1->sFilterRegister[filterIDpointer/4].FR2 |= id << 21;
				break;
	 }
	 filterIDpointer++;
	 CAN1->FMR   &= ~(0x1UL);			  // Deactivate initialization mode
	 
	 return 0;
 }
 
void STM32L4_CAN::CANReceive(CAN_msg_t *CAN_rx_msg){
	CAN_rx_msg->id  = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FFUL;
	CAN_rx_msg->len = (CAN1->sFIFOMailBox[0].RDTR) & 0xFUL;
	
	CAN_rx_msg->data[0] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDLR;
	CAN_rx_msg->data[1] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 8);
	CAN_rx_msg->data[2] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 16);
	CAN_rx_msg->data[3] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 24);
	CAN_rx_msg->data[4] = 0xFFUL &  CAN1->sFIFOMailBox[0].RDHR;
	CAN_rx_msg->data[5] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 8);
	CAN_rx_msg->data[6] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 16);
	CAN_rx_msg->data[7] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 24);
	
	CAN1->RF0R |= 0x20UL;
 }
 
uint8_t STM32L4_CAN::CANSend(CAN_msg_t *CAN_tx_msg){
	int count = 0;
	 
	CAN1->sTxMailBox[0].TIR   = (CAN_tx_msg->id) << 21;
	
	CAN1->sTxMailBox[0].TDTR &= ~(0xF);
	CAN1->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;
	
	CAN1->sTxMailBox[0].TDLR  = (((uint32_t) CAN_tx_msg->data[3] << 24) |
								 ((uint32_t) CAN_tx_msg->data[2] << 16) |
								 ((uint32_t) CAN_tx_msg->data[1] <<  8) |
								 ((uint32_t) CAN_tx_msg->data[0]      ));
	CAN1->sTxMailBox[0].TDHR  = (((uint32_t) CAN_tx_msg->data[7] << 24) |
								 ((uint32_t) CAN_tx_msg->data[6] << 16) |
								 ((uint32_t) CAN_tx_msg->data[5] <<  8) |
								 ((uint32_t) CAN_tx_msg->data[4]      ));

	CAN1->sTxMailBox[0].TIR  |= 0x1UL;
	while(CAN1->sTxMailBox[0].TIR & 0x1UL && count++ < 1000000);
	 
	 if (!(CAN1->sTxMailBox[0].TIR & 0x1UL)) return 0;
	 
	 //Sends error log to screen
	 while (CAN1->sTxMailBox[0].TIR & 0x1UL)
	 {
		//Error!
		return 1;
		/*
		 SendInt(CAN1->ESR);
		 SendLine();
		 SendInt(CAN1->MSR);
		 SendLine();
		 SendInt(CAN1->TSR);
		 SendLine();
		 SendLine();
		 */
	 }
 }

uint8_t STM32L4_CAN::CANMsgAvail(void){
	 return CAN1->RF0R & 0x3UL;
}
