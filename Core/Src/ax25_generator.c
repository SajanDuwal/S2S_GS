/*
 * ax25_generator.c
 *
 *  Created on: Jun 20, 2024
 *      Author: sajanduwal
 */

#include "ax25_generator.h"

#define CMD_PAYLOAD	(35)

extern uint8_t tx_cmd[CMD_PAYLOAD];

void AX_25PacketFormation(uint8_t *main_cmd) {

	myDebug("AX.25 information field: \r\n");
	for (int i = 0; i < 13; i++) {
		myDebug("%x ", main_cmd[i]);
	}
	myDebug("\r\n");

	uint8_t buff_head[17];
	// AX.25 Packet header
	tx_cmd[0] = 0x73;

	// destination callsign
	tx_cmd[1] = 0x72;    // callsign  9
	tx_cmd[2] = 0x9C;	//callsign	N
	tx_cmd[3] = 0x64;	//callsign	2
	tx_cmd[4] = 0xA6;	//callsign	S
	tx_cmd[5] = 0x92;	//callsign	I
	tx_cmd[6] = 0x40;	// callsign    space 0x20 -> 1bit left shift 0x40
	tx_cmd[7] = 0xE0;	// destination SSID

	// source callsign					Sajan
	tx_cmd[8] = 0x53;    // callsign S
	tx_cmd[9] = 0x41;	//callsign	A
	tx_cmd[10] = 0x4A;	//callsign	J
	tx_cmd[11] = 0x41;	//callsign	A
	tx_cmd[12] = 0x4E;	//callsign	N
	tx_cmd[13] = 0x40;	// callsign space
	tx_cmd[14] = 0x36;	// source SSID

	// control field
	tx_cmd[15] = 0x03;

	// PID control bit
	tx_cmd[16] = 0xF0;

	for (int a = 0; a < 17; a++) {
		buff_head[a] = tx_cmd[a];
	}

	// Calculate CRC-CCITT for the packet data starting from tx_cmd[0] to tx_cmd[16]

	uint16_t crc = 0;
	crc = calculateCRC_CCITT_AX25(buff_head, sizeof(buff_head));

	tx_cmd[17] = (crc >> 8) && 0xFF;
	tx_cmd[18] = crc & 0xFF;

	// information field
	int i = 19;
	for (int k = 0; k < 13; k++) {
		tx_cmd[i] = main_cmd[k];
		i++;
	}

	// Calculate CRC-CCITT for the packet data starting from packet[1]
	crc = 0;
	crc = calculateCRC_CCITT_AX25(main_cmd, 13);

	memset(main_cmd, '\0', 13);

	// Store CRC result in the packet array (from packet[1] to end of for loop)
	tx_cmd[i] = (crc >> 8) & 0xFF; // Most significant byte
	i++;
	tx_cmd[i] = crc & 0xFF;        // Least significant byte
	i++;
	// AX.25 Packet footer
	tx_cmd[i] = 0x73;
	myDebug("\npacket_len: %d\r\n", i + 1);
	myDebug("packet: 0x%x\r\n", tx_cmd);
	for (int j = 0; j <= i; j++) {
		myDebug("%x ", tx_cmd[j]);
	}
	myDebug("\r\n");
}
