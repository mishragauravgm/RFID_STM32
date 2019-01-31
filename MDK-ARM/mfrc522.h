/*
 * mfrc522.h
 * 
 * Copyright 2018 Gaurav Mishra <gm213@snu.edu.in>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

#define MFRC522_H

#include "stdint.h"
#include "mfrc522_cmd.h"
#include "mfrc522_reg.h"


typedef enum{
	MI_OK = 0,
	MI_NOTAGERR,
	MI_ERR} mfrc522_status_t;

#define CARD_FOUND			1
#define CARD_NOT_FOUND	2
#define ERROR						3

#define MAX_LEN					16

//Card types
#define Mifare_UltraLight 	0x4400
#define Mifare_One_S50			0x0400
#define Mifare_One_S70			0x0200
#define Mifare_Pro_X				0x0800
#define Mifare_DESFire			0x4403

// Mifare_One card command word
# define PICC_REQIDL          0x26               // find the antenna area does not enter hibernation
# define PICC_REQALL          0x52               // find all the cards antenna area
# define PICC_ANTICOLL        0x93               // anti-collision
# define PICC_SElECTTAG       0x93               // election card
# define PICC_AUTHENT1A       0x60               // authentication key A
# define PICC_AUTHENT1B       0x61               // authentication key B
# define PICC_READ            0x30               // Read Block
# define PICC_WRITE           0xA0               // write block
# define PICC_DECREMENT       0xC0               // debit
# define PICC_INCREMENT       0xC1               // recharge
# define PICC_RESTORE         0xC2               // transfer block data to the buffer
# define PICC_TRANSFER        0xB0               // save the data in the buffer
# define PICC_HALT            0x50               // Sleep

void mfrc522_init(void);
mfrc522_status_t check(uint8_t*);
void mfrc522_reset(void);
void mfrc522_writeRegister(uint8_t, uint8_t);
uint8_t mfrc522_readRegister(uint8_t);
uint8_t	mfrc522_Request(uint8_t, uint8_t *);
mfrc522_status_t mfrc522_toCard(uint8_t, uint8_t*, uint8_t, uint8_t* , uint16_t*);
void antennaOn(void);
void antennaOff(void);
uint8_t mfrc522_get_card_serial(uint8_t*);
void 	mfrc522_halt(void);
mfrc522_status_t mfrc522_AntiColl(uint8_t*);
mfrc522_status_t mfrc522_check(uint8_t*);