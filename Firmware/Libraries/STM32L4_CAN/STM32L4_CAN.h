/**
 * This is the header file for the CAN driver.
 *
ar * 
 * Polling will be required by the user, since the functions executed when CAN 
 * messages are received are too complex and long, and will not be suitable to
 * be placed in a interrupt handler (the alternative is to have the interrupt 
 * set a valid bit and poll that bit in the main loop. Unfortunately, clearing
 * the interrupt mask means setting the pending number of CAN messages to 0, 
 * which means, depending on the rate of messages being received, some messages
 * will be dropped.)
 *
 * 
 * based on this code: https://github.com/UBC-Solar/Firmware-v2/tree/master/Peripherals
 */

#ifndef STM32L4_CAN_H
#define STM32L4_CAN_H

#include <stdio.h>
#include <stm32l433xx.h>


enum BITRATE{CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS};

typedef struct{
	uint16_t id;
	uint8_t  data[8];
	uint8_t  len;
} CAN_msg_t;

typedef const struct
{
	uint8_t TS2;
	uint8_t TS1;
	uint8_t BRP;
} CAN_bit_timing_config_t;

class STM32L4_CAN{
	public:
		STM32L4_CAN(BITRATE bitrate = CAN_500KBPS); //init with given baud
		uint8_t  CANAddFilter(uint16_t id);			//Add id to the filter list
		void CANReceive(CAN_msg_t *CAN_rx_msg); 	//Get message from mailbox
		uint8_t CANSend(CAN_msg_t *CAN_tx_msg);    	//Add message to mailbox
		uint8_t CANMsgAvail(void);					//Get number of messages waiting to process
	private:
		uint32_t filterIDpointer = 0;
		CAN_bit_timing_config_t can_configs[6] = {{2, 13, 45}, {2, 15, 20}, {2, 13, 18}, {2, 13, 9}, {2, 15, 4}, {2, 15, 2}};
};
#endif