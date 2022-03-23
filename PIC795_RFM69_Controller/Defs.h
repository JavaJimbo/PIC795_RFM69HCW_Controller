/* 
 * "Defs.h"
 * Include file for PIC795_RFM69_Controller project
 * Author: Jim Sedgwick
 *
 * Created on March 23, 2022
 */
#include <plib.h>

#ifndef DEFS_H
#define	DEFS_H

// #define byte unsigned char
// #define BYTE byte

#define SYS_FREQ 80000000
#define GetPeripheralClock() SYS_FREQ

#define RF69_FREQ 915.0
#define true TRUE
#define false FALSE
#define uint8_t unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned long
#define int8_t char
#define int16_t short
#define int32_t long

#define RF69_CHANNEL SPI_CHANNEL2

#ifndef _MAIN
    extern unsigned long milliseconds;    
#endif
    
#define RF69_FREQ 915.0    
#define MY_ADDRESS 1    
#define DESTINATION_ADDRESS 1    
    
#define millis() milliseconds
    
#define RFM69_CS_HIGH() LATDbits.LATD6 = 1 // PORTSetBits(PORTD, BIT_6)
#define RESET_HIGH() LATDbits.LATD7 = 1

#define RFM69_CS_LOW() LATDbits.LATD6 = 0 // PORTClearBits(PORTD, BIT_6)
#define RESET_LOW() LATDbits.LATD7 = 0
#define TEST_OUT LATCbits.LATC1    
    
    
#endif	/* DEFS_H */

