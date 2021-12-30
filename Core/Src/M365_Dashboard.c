/*
 * M365_Dashboard.c
 *
 *  Created on: Nov 27, 2021
 *      Author: stancecoke
 */


#include "main.h"
#include "config.h"
#include "stm32f1xx_hal.h"
#include "print.h"
#include "M365_Dashboard.h"
#include "M365_memory_table.h"
enum { STATE_LOST, STATE_START_DETECTED, STATE_LENGTH_DETECTED };

UART_HandleTypeDef huart3;
static uint8_t ui8_rx_buffer[140];
static uint8_t ui8_dashboardmessage[140];
static uint8_t	ui8_tx_buffer[96];// = {0x55, 0xAA, 0x08, 0x21, 0x64, 0x00, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t ui8_oldpointerposition=64;
static uint8_t ui8_recentpointerposition=0;
static uint8_t ui8_messagestartpos=255;
static uint8_t ui8_messagelength=0;
static uint8_t ui8_state= STATE_LOST;
static uint32_t ui32_timeoutcounter=0;

char *target;
char *source;
static uint8_t ui8_target_offset;
static uint8_t ui8_source_offset;

M365_menory_table_t MT;

enum bytesOfMessage65 {
	Throttle = 7,
	Brake = 8,
} msg65;

enum bytesOfMessage64 {
	Speed = 10,
	Mode = 6,
	SOC = 7,
	Light = 8,
	Beep = 9
} msg64;

enum bytesOfGeneralMessage {
	msglength = 2,
	receiver = 3,
	command = 4,
	startAddress = 5,
	payloadLength = 6
} gen_msg;


void M365Dashboard_init(UART_HandleTypeDef huart1) {
//        CLEAR_BIT(huart3.Instance->CR3, USART_CR3_EIE);
	if (HAL_UART_Receive_DMA(&huart1, (uint8_t*) ui8_rx_buffer, sizeof(ui8_rx_buffer)) != HAL_OK) {
		Error_Handler();
	}
	ui8_tx_buffer[0] = 0x55;
	ui8_tx_buffer[1] = 0xAA;
	MT.ESC_version = 0x0189;
	MT.internal_battery_version = 0x0289;
	MT.total_riding_time[0]=0xFFFF;
	strcpy(MT.scooter_serial, "Stancecoke_1");
	MT.ESC_status_2= 0x0800;

}

void search_DashboardMessage(M365State_t* p_M365State, UART_HandleTypeDef huart1){

	if(ui32_timeoutcounter>3200){
		if (HAL_UART_Receive_DMA(&huart1, (uint8_t*) ui8_rx_buffer, sizeof(ui8_rx_buffer)) != HAL_OK) {
			Error_Handler();
		}
		ui32_timeoutcounter=0;
		printf_("DMA Receive timeout! \n");
	}

	ui8_recentpointerposition = sizeof(ui8_rx_buffer) - (DMA1_Channel5->CNDTR); //Pointer of UART1RX DMA Channel
		if (ui8_recentpointerposition<ui8_oldpointerposition){
			ui8_oldpointerposition=ui8_recentpointerposition-1;
			ui8_state=STATE_LOST;
		}
		while(ui8_oldpointerposition!=ui8_recentpointerposition){

			switch (ui8_state) {
			case STATE_LOST: { //if no message start is detected yet, search for start pattern 0x55 0xAA
				if(ui8_rx_buffer[ui8_oldpointerposition]==0xAA&&ui8_rx_buffer[ui8_oldpointerposition-1]==0x55){
					ui8_messagestartpos=ui8_oldpointerposition-1;
					ui8_state=STATE_START_DETECTED;
				}
			}
				break;

			case STATE_START_DETECTED: { //read the lenght of the message
				if(ui8_oldpointerposition==ui8_messagestartpos+2){
					ui8_messagelength=ui8_rx_buffer[ui8_oldpointerposition]+6;
					ui8_state=STATE_LENGTH_DETECTED;
				}
			}
				break;
			case STATE_LENGTH_DETECTED: { //read whole message and call processing
				if(ui8_oldpointerposition==ui8_messagestartpos+ui8_messagelength-1){
					memcpy(ui8_dashboardmessage,ui8_rx_buffer+ui8_messagestartpos,ui8_messagelength);
					process_DashboardMessage(p_M365State, (uint8_t*)&ui8_dashboardmessage,ui8_messagelength,huart1);
					ui8_state=STATE_LOST;
				  	   CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
				  	   DMA1_Channel5->CNDTR=sizeof(ui8_rx_buffer);
				  	   SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
				  	   ui32_timeoutcounter=0;


				}
			}
				break;
			} //end switch

			ui8_oldpointerposition=(ui8_oldpointerposition+1)% sizeof(ui8_rx_buffer);
		}// end of while
		ui32_timeoutcounter++;
}

void process_DashboardMessage(M365State_t* p_M365State, uint8_t *message, uint8_t length, UART_HandleTypeDef huart1) {
	//while(HAL_UART_GetState(&huart1)!=HAL_UART_STATE_READY){}
	//HAL_Delay(2); // bad style, but wait for characters coming in, if message is longer than expected
	if(!checkCRC(message, length)){
	//	55 AA 06 21 64 00 00 00 00 00 74 FF
	//55	AA	8	21	64	0	20	0	0	1	0	12	3F	FF

		switch (message[command]) {

		case 0x64: {

			ui8_tx_buffer[5]=0x00;
			ui8_tx_buffer[11]=0x00;
			ui8_tx_buffer[msglength]=0x08;
			ui8_tx_buffer[receiver]=0x21;
			ui8_tx_buffer[command]=message[command];
			ui8_tx_buffer[Speed]=p_M365State->speed;
			ui8_tx_buffer[Mode]=p_M365State->mode;
			ui8_tx_buffer[SOC]=map(p_M365State->Voltage,33000,42000,0,96);
			if(p_M365State->light)ui8_tx_buffer[Light]=64;
			else ui8_tx_buffer[Light]=0;
			ui8_tx_buffer[Beep]= p_M365State->beep;

			addCRC((uint8_t*)ui8_tx_buffer, ui8_tx_buffer[msglength]+6);
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ui8_tx_buffer, ui8_tx_buffer[msglength]+6);
			if(p_M365State->beep&&ui8_tx_buffer[Beep])p_M365State->beep = 0;

			}
			break;

		case 0x65: {
			if(map(message[Brake],BRAKEOFFSET,BRAKEMAX,0,REGEN_CURRENT)>0){
				if(p_M365State->speed > 2)	p_M365State->i_q_setpoint_target = -map(message[Brake],BRAKEOFFSET,BRAKEMAX,0,REGEN_CURRENT);
				else p_M365State->i_q_setpoint_target = 0;
				}
			else{
				p_M365State->i_q_setpoint_target = map(message[Throttle],THROTTLEOFFSET,THROTTLEMAX,0,p_M365State->phase_current_limit);
				}
			}
			break;

		case 0x61: {
			//55 AA 06 20 61 DA 0C 02 27 00 69 FE
			//55 AA 0E 23 01 DA 48 FF 73 06 78 78 54 51 53 32 10 67 A2 FA

			if(map(message[9],BRAKEOFFSET,BRAKEMAX,0,REGEN_CURRENT)>0){
				if(p_M365State->speed > 2) p_M365State->i_q_setpoint_target = -map(message[9],BRAKEOFFSET,BRAKEMAX,0,REGEN_CURRENT);
				else p_M365State->i_q_setpoint_target = 0;
				}
			else{
				p_M365State->i_q_setpoint_target = map(message[8],THROTTLEOFFSET,THROTTLEMAX,0,p_M365State->phase_current_limit);
				}


			ui8_tx_buffer[msglength]=message[payloadLength]+2;
			ui8_tx_buffer[receiver]=message[receiver]+3;
			ui8_tx_buffer[command]=0x01;
			ui8_tx_buffer[startAddress] =message[startAddress];

			source = (char *)&MT;
			target = (char *)&ui8_tx_buffer;
			ui8_source_offset = message[startAddress];
			ui8_target_offset = 6;
			memcpy(target+ui8_target_offset,source+ui8_source_offset*2,message[payloadLength]);
			addCRC((uint8_t*)ui8_tx_buffer, ui8_tx_buffer[msglength]+6);
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ui8_tx_buffer, ui8_tx_buffer[msglength]+6);
			}
			break;

		case 0x03: {
			//55	AA	4	20	3	70	1	0	67	FF

			source = (char *)&message;
			target = (char *)&MT;
			ui8_target_offset = message[startAddress];

			memcpy(target+ui8_target_offset,source+6,1);
			if (message[payloadLength]==1)MT.ESC_status_2= 0x0802;

			}
			break;

		case 0x0A: {
			NVIC_SystemReset();

			}
			break;

		case 0x07: {
			//55 AA 06 20 07 00 08 61 00 00 69 FF
			//55 AA 02 23 07 00 D3 FF
			ui8_tx_buffer[msglength] = 2;
			ui8_tx_buffer[receiver]=message[receiver]+3;
			ui8_tx_buffer[command]=0x07;
			ui8_tx_buffer[startAddress] =0;
			addCRC((uint8_t*)ui8_tx_buffer, ui8_tx_buffer[msglength]+6);
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ui8_tx_buffer, ui8_tx_buffer[msglength]+6);
			}
			break;

		case 0x08: {
			//55 AA 42 20 08 00 FA 8B 7B 71 4F 4C 97 16 0B 71 34 89 96 24 DE C2 3B 3E FF 06 B7 3B 69 69 BB 8A 56 10 A0 A0 34 E0 15 65 D2 6E 04 62 4C EB BB 6B 49 C1 7F F6 EA B7 64 F7 AD 5D 4E 8C D1 2C DB 7E EC B6 F4 73 FC A3 2E DE
			//55 AA 02 23 08 00 D2 FF
			ui8_tx_buffer[msglength] = 2;
			ui8_tx_buffer[receiver]=message[receiver]+3;
			ui8_tx_buffer[command]=0x08;
			ui8_tx_buffer[startAddress] =0;
			addCRC((uint8_t*)ui8_tx_buffer, ui8_tx_buffer[msglength]+6);
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ui8_tx_buffer, ui8_tx_buffer[msglength]+6);
			}
			break;

		case 0x09: {
			//55	AA	6	20	9	0	91	9E	CF	FF	D3	FC
			//55	AA	2	23	9	0	D1	FF
			ui8_tx_buffer[msglength] = 2;
			ui8_tx_buffer[receiver]=message[receiver]+3;
			ui8_tx_buffer[command]=0x09;
			ui8_tx_buffer[startAddress] =0;
			addCRC((uint8_t*)ui8_tx_buffer, ui8_tx_buffer[msglength]+6);
			HAL_HalfDuplex_EnableTransmitter(&huart1);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ui8_tx_buffer, ui8_tx_buffer[msglength]+6);
			}
			break;

		default: {
		//	MS->i_q_setpoint = 0; // stop motor for safety reason
			}
			break;
		}//end switch
	}
}

void addCRC(uint8_t * message, uint8_t size){
    unsigned long cksm = 0;
    for(int i = 2; i < size - 2; i++) cksm += message[i];
    cksm ^= 0xFFFF;
    message[size - 2] = (uint8_t)(cksm&0xFF);
    message[size - 1] = (uint8_t)((cksm&0xFF00) >> 8);
    message[size] = '\0';
}

int16_t checkCRC(uint8_t * message, uint8_t size){
    unsigned long cksm = 0;
    for(int i = 2; i < size - 2; i++) cksm += message[i];
    cksm ^= 0xFFFF;
    return cksm-(message[size - 2]+(message[size - 1]<<8));
}
