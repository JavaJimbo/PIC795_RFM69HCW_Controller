/**********************************************************************************
 * PROJECT: PIC795 RFM69 CONTROLLER * 
 * Sends and receives wireless data using RFM69HCW transceiver module
 * Uses routines adapted from Adafruit version of Radiohead library
 *
 * main.c
 * Compiled for PIC32MX795 XC32 compiler version 1.30 
 * 
 * 3-22-22:     Got Adafruit RFM69HCW modules receiving!!!!
 * 3-23-22:     Transmitting works too!
 ***********************************************************************************/
#include <plib.h>
#include "Defs.h"
#include "Delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "Compiler.h"
#include "I2C_4BUS_EEPROM_PIC32.h"
#include "RH_RF69.h"
#include "RF69_Robotnik.h"

#define _SUPPRESS_PLIB_WARNING

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer enabled
#pragma config WDTPS =    PS8192        // Watchdog Timer Postscaler (1:8192) For 31,250 clock divided by 8192 = 262 mS timeout
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

#define false FALSE
#define true TRUE

uint8_t arrReceiveMessage[RH_RF69_MAX_MESSAGE_LEN];

extern volatile uint16_t    _txGood;
extern volatile int16_t     _lastRssi;
extern uint32_t            _lastPreambleTime;
extern volatile RHMode     _mode;


#define	STX '>'
#define	DLE '/'
#define	ETX '\r'


#define NUM_LOCAL_POTS 4

#define MAXPOTS 4

#define	STX '>'
#define	DLE '/'
#define	ETX '\r'


#define EEPROM_WRITE_PROTECT LATDbits.LATD8


#ifdef USE_BRAIN_BOARD
    #define LED LATEbits.LATE6
#else
    #define LED LATDbits.LATD9
#endif

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

unsigned long milliseconds;

uint8_t HOSTRxBuffer[RH_RF69_MAX_MESSAGE_LEN+1];
uint8_t HOSTRxLength = 0;
BYTE HOSTRxBufferFull = false;
BYTE HOSTTxBuffer[RH_RF69_MAX_MESSAGE_LEN+1]; 


int timeout = 0;

extern BYTE CheckCRC (BYTE *ptrPacketData, short packetLength);

int ADC10_ManualInit(void);
static void InitializeSystem(void);
extern BYTE CheckCRC (BYTE *ptrRxModbus, short RxModbusLength);


unsigned short ADresult[MAXPOTS];
unsigned short PORTB_Read = 0, PORTD_Read = 0;    
BYTE RFM69_IntFlag = false;    
BYTE RFM69_INT_State = 0;
long RFM69InterruptCounter = 0;
    
BYTE intFlag = false;

long ActualHOSTBaudRate;

int main(void) 
{
    uint8_t NumReceivedBytes = 0;
    uint8_t packetnum = 0;
        
         
    DelayMs(100);           
    InitializeSystem();
    // Manual reset 
    RESET_HIGH();
    DelayMs(10);
    RESET_LOW();
    DelayMs(10);
    
    printf("\r\r#1 INITIALIZING RFM69HCW Module...");
    if (!RH_RF69_init()) 
    {        
        printf("\rRFM69 INITIALIZATION ERROR");
        while(1) ClrWdt(); // CLEAR_WATCHDOG
    }
    printf("\rSUCCESS - RFM69 INITIAILIZED");
 
    setFrequency(RF69_FREQ);
    
    // If you are using a high power RF69 eg RFM69HW, 
    // use range from 14-20 for power and flag set true:
    setTxPower(20, true);  
    
 
    
  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
   setEncryptionKey(key);    
    
       
        
#ifdef USE_BRAIN_BOARD    
    printf("\r\rSTART BRAIN BOARD VERSION #0 NO RC SERVOS");
#else
    printf("\r\rRFM69HCW Module receive and transmit testing\r");
#endif
    
    while(1) 
    {   
        ClrWdt(); // CLEAR_WATCHDOG

        if (available())
        {
            getRxMessage (arrReceiveMessage, &NumReceivedBytes);
            arrReceiveMessage[NumReceivedBytes] = '\0';
            printf("\rReceived %d bytes: %s", NumReceivedBytes, arrReceiveMessage);
        }
    
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false;                                 
            // Send packet to the DESTINATION!
            setHeaderId(packetnum++);  
            sendto(HOSTRxBuffer, HOSTRxLength, DESTINATION_ADDRESS);
            waitPacketSent(200);   
            HOSTRxBuffer[HOSTRxLength] = '\0';
            printf("\rSent %d bytes: %s", HOSTRxLength, HOSTRxBuffer);
        }
    } // End while(1))
} // End main())




// Timer 2 generates an interrupt every 50 microseconds approximately
void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{        
    static int intCounter = 0;  
    static int millisecondCounter = 20;    
    
    mT2ClearIntFlag(); // clear the interrupt flag     
    
    if (TEST_OUT) TEST_OUT = 0;
    else TEST_OUT = 1;
    
    if (millisecondCounter) millisecondCounter--;
    if (millisecondCounter <= 0)
    {        
        millisecondCounter = 20;
        milliseconds++;
    }    
    
    intCounter++;
    if (intCounter >= 80)// -80) // 2000
    {
        intCounter = 0;
        intFlag = true;
    }
    
}








#ifdef USE_BRAIN_BOARD
// When using Brain Board: Analog inputs 1-4 are diagnostic pot inputs and inputs 12-15 are rotary pot inputs
int ADC10_ManualInit(void)
{
    int i, dummy;
    
    AD1CON1bits.ON = 0;
    mAD1IntEnable(INT_DISABLED);   
    mAD1ClearIntFlag();
    
    AD1CON1 = 0;
    AD1CON2 = 0;
    AD1CON3 = 0;
    AD1CHS  = 0;
    AD1CSSL = 0;
    
    // Set each Port B pin for digital or analog
    // Analog = 0, digital = 1
    AD1PCFGbits.PCFG0 = 1; 
    
    AD1PCFGbits.PCFG1 = 0; 
    AD1PCFGbits.PCFG2 = 0; 
    AD1PCFGbits.PCFG3 = 0; 
    AD1PCFGbits.PCFG4 = 0; 
    
    AD1PCFGbits.PCFG5 = 1; 
    AD1PCFGbits.PCFG6 = 1; 
    AD1PCFGbits.PCFG7 = 1; 
    
    AD1PCFGbits.PCFG8 = 1; 
    AD1PCFGbits.PCFG9 = 1; 
    AD1PCFGbits.PCFG10 = 1; 
    AD1PCFGbits.PCFG11 = 1;
    
    AD1PCFGbits.PCFG1 = 0; 
    AD1PCFGbits.PCFG2 = 0; 
    AD1PCFGbits.PCFG3 = 0; 
    AD1PCFGbits.PCFG4 = 0; 
    
    AD1PCFGbits.PCFG5 = 1; 
    AD1PCFGbits.PCFG6 = 1; 
    AD1PCFGbits.PCFG7 = 1; 
    
    AD1PCFGbits.PCFG8 = 1; 
    AD1PCFGbits.PCFG9 = 1; 
    AD1PCFGbits.PCFG10 = 1; 
    AD1PCFGbits.PCFG11 = 1;
    
    
    AD1PCFGbits.PCFG12 = 0; 
    AD1PCFGbits.PCFG13 = 0; 
    AD1PCFGbits.PCFG14 = 0; 
    AD1PCFGbits.PCFG15 = 0;     
    
    AD1CON1bits.FORM = 000;        // 16 bit integer format.
    AD1CON1bits.SSRC = 7;        // Auto Convert
    AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
    AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
    
    AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
    AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
    AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
    AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
    AD1CON2bits.BUFM = 0;        // One 16 word buffer
    AD1CON2bits.ALTS = 0;        // Use only Mux A
    AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
    AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
    AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
    AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3

    // Set conversion clock and set sampling time.
    AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
    AD1CON3bits.SAMC = 0b11111;        // Sample time max
    AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

    // Select channels to scan. Scan channels = 1, Skip channels = 0
    AD1CSSLbits.CSSL0 = 0;
    
    AD1CSSLbits.CSSL1 = 1;
    AD1CSSLbits.CSSL2 = 1;
    AD1CSSLbits.CSSL3 = 1;    
    AD1CSSLbits.CSSL4 = 1;
    
    AD1CSSLbits.CSSL5 = 0;
    AD1CSSLbits.CSSL6 = 0;
    AD1CSSLbits.CSSL7 = 0;
    AD1CSSLbits.CSSL8 = 0;
    AD1CSSLbits.CSSL9 = 0;
    AD1CSSLbits.CSSL10 = 0;
    AD1CSSLbits.CSSL11 = 0;
    
    AD1CSSLbits.CSSL12 = 1;
    AD1CSSLbits.CSSL13 = 1;
    AD1CSSLbits.CSSL14 = 1;
    AD1CSSLbits.CSSL15 = 1;
    
    // Make sure all buffers have been Emptied. 
    for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);    
    
    AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
    AD1CON1bits.ON = 1;            // Turn on ADC.
    return (1);
}

#else
    #ifndef REV2
    // When using REV 1 MD13S Grove controller board:
    // Analog input Analog inputs 1-4 are diagnostic pot inputs and inputs 12-15 are rotary pot inputs
    int ADC10_ManualInit(void)
    {
        int i, dummy;
    
        AD1CON1bits.ON = 0;
        mAD1IntEnable(INT_DISABLED);   
        mAD1ClearIntFlag();
    
        AD1CON1 = 0;
        AD1CON2 = 0;
        AD1CON3 = 0;
        AD1CHS  = 0;
        AD1CSSL = 0;
    
        // Set each Port B pin for digital or analog
        // Analog = 0, digital = 1
        AD1PCFGbits.PCFG0 = 1; 
        AD1PCFGbits.PCFG1 = 1; 
        AD1PCFGbits.PCFG2 = 1; 
        AD1PCFGbits.PCFG3 = 0; 
        AD1PCFGbits.PCFG4 = 0; 
        AD1PCFGbits.PCFG5 = 1; 
        AD1PCFGbits.PCFG6 = 1; 
        AD1PCFGbits.PCFG7 = 1; 
        AD1PCFGbits.PCFG8 = 0; 
        AD1PCFGbits.PCFG9 = 0; 
        AD1PCFGbits.PCFG10 = 0; 
        AD1PCFGbits.PCFG11 = 0; 
        AD1PCFGbits.PCFG12 = 0; 
        AD1PCFGbits.PCFG13 = 0; 
        AD1PCFGbits.PCFG14 = 0; 
        AD1PCFGbits.PCFG15 = 0;     
    
        AD1CON1bits.FORM = 000;        // 16 bit integer format.
        AD1CON1bits.SSRC = 7;        // Auto Convert
        AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
        AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
        
        AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
        AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
        AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
        AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
        AD1CON2bits.BUFM = 0;        // One 16 word buffer
        AD1CON2bits.ALTS = 0;        // Use only Mux A
        AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
        AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
        AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
        AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3
        
        // Set conversion clock and set sampling time.
        AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
        AD1CON3bits.SAMC = 0b11111;        // Sample time max
        AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

        // Select channels to scan. Scan channels = 1, Skip channels = 0
        AD1CSSLbits.CSSL0 = 0;
        AD1CSSLbits.CSSL1 = 0;
        AD1CSSLbits.CSSL2 = 0;
        AD1CSSLbits.CSSL3 = 1;
        AD1CSSLbits.CSSL4 = 1;
        AD1CSSLbits.CSSL5 = 0;
        AD1CSSLbits.CSSL6 = 0;
        AD1CSSLbits.CSSL7 = 0;
        AD1CSSLbits.CSSL8 = 1;
        AD1CSSLbits.CSSL9 = 1;
        AD1CSSLbits.CSSL10 = 1;
        AD1CSSLbits.CSSL11 = 1;
        AD1CSSLbits.CSSL12 = 1;
        AD1CSSLbits.CSSL13 = 1;
        AD1CSSLbits.CSSL14 = 1;
        AD1CSSLbits.CSSL15 = 1;
        
        // Make sure all buffers have been Emptied. 
        for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);    
        
        AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
        AD1CON1bits.ON = 1;            // Turn on ADC.
        return (1);
    }
    #else
    // When using REV 2 MD13S Grove controller board:
    // Analog input Analog inputs 1-4 are diagnostic pot inputs and inputs 12-15 are rotary pot inputs
    int ADC10_ManualInit(void)
    {
        int i, dummy;
    
        AD1CON1bits.ON = 0;
        mAD1IntEnable(INT_DISABLED);   
        mAD1ClearIntFlag();
    
        AD1CON1 = 0;
        AD1CON2 = 0;
        AD1CON3 = 0;
        AD1CHS  = 0;
        AD1CSSL = 0;
    
        // Set each Port B pin for digital or analog
        // Analog = 0, digital = 1
        AD1PCFGbits.PCFG0 = 1; 
        AD1PCFGbits.PCFG1 = 1; 
        AD1PCFGbits.PCFG2 = 1; 
        AD1PCFGbits.PCFG3 = 1; 
        AD1PCFGbits.PCFG4 = 1; 
        AD1PCFGbits.PCFG5 = 1; 
        AD1PCFGbits.PCFG6 = 1; 
        AD1PCFGbits.PCFG7 = 1; 
        AD1PCFGbits.PCFG8 = 0; 
        AD1PCFGbits.PCFG9 = 0; 
        AD1PCFGbits.PCFG10 = 0; 
        AD1PCFGbits.PCFG11 = 0;         
        AD1PCFGbits.PCFG12 = 0; 
        AD1PCFGbits.PCFG13 = 0; 
        AD1PCFGbits.PCFG14 = 0; 
        AD1PCFGbits.PCFG15 = 0;     
    
        AD1CON1bits.FORM = 000;        // 16 bit integer format.
        AD1CON1bits.SSRC = 7;        // Auto Convert
        AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
        AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
        
        AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
        AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
        AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
        AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
        AD1CON2bits.BUFM = 0;        // One 16 word buffer
        AD1CON2bits.ALTS = 0;        // Use only Mux A
        AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
        AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
        AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
        AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3
        
        // Set conversion clock and set sampling time.
        AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
        AD1CON3bits.SAMC = 0b11111;        // Sample time max
        AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

        // Select channels to scan. Scan channels = 1, Skip channels = 0
        AD1CSSLbits.CSSL0 = 0;
        AD1CSSLbits.CSSL1 = 0;
        AD1CSSLbits.CSSL2 = 0;
        AD1CSSLbits.CSSL3 = 0;
        AD1CSSLbits.CSSL4 = 0;
        AD1CSSLbits.CSSL5 = 0;
        AD1CSSLbits.CSSL6 = 0;
        AD1CSSLbits.CSSL7 = 0;
        AD1CSSLbits.CSSL8 = 1;
        AD1CSSLbits.CSSL9 = 1;
        AD1CSSLbits.CSSL10 = 1;
        AD1CSSLbits.CSSL11 = 1;
        AD1CSSLbits.CSSL12 = 1;
        AD1CSSLbits.CSSL13 = 1;
        AD1CSSLbits.CSSL14 = 1;
        AD1CSSLbits.CSSL15 = 1;
        
        // Make sure all buffers have been Emptied. 
        for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);    
        
        AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
        AD1CON1bits.ON = 1;            // Turn on ADC.
        return (1);
    }
    #endif
#endif

void InitializeSystem(void) 
{	
    SYSTEMConfigPerformance(80000000);
    
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    // Configure PIC ADC for ten AD input channels
    ADC10_ManualInit();    
    
    // Set up UART Rx DMA interrupts
    // SetupDMA_Rx();
    
    // I/O Ports:
#ifdef USE_BRAIN_BOARD
    PORTSetPinsDigitalOut(IOPORT_A, BIT_5);  // MOTOR_DIR4     
    PORTSetPinsDigitalOut(IOPORT_B, BIT_0);  // RS485_ENABLE
    PORTSetPinsDigitalOut(IOPORT_D, BIT_8 | BIT_9 | BIT_11 | BIT_13);  // EE_WR, LED, MOTOR DIR #3, #2
    PORTSetPinsDigitalIn(IOPORT_E, BIT_9 | BIT_5 | BIT_7 | BIT_8 | BIT_6);  // ENCODER DIRECTION INPUTS 1-4 
    PORTSetPinsDigitalOut(IOPORT_E, BIT_6);  // LED
    PORTSetPinsDigitalOut(IOPORT_F, BIT_1);  // PWM DISABLE
    PWM_DISABLE = 0;
    PORTSetPinsDigitalOut(IOPORT_G, BIT_12);  // MOTOR DIR #1           
#else    
    #ifndef REV2   
        PORTSetPinsDigitalOut(IOPORT_A, BIT_5);  // MOTOR_DIR4     
        PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_2);  // SW1-3
        PORTSetPinsDigitalOut(IOPORT_B, BIT_5);  // RS485_ENABLE
        PORTSetPinsDigitalIn(IOPORT_C, BIT_13);  // SW4
        PORTSetPinsDigitalOut(IOPORT_C, BIT_1);  // TEST_OUT        
        PORTSetPinsDigitalIn(IOPORT_D, BIT_4 | BIT_5);  // SW5
        PORTSetPinsDigitalOut(IOPORT_D, BIT_6 | BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_13);  // RFM69_CS, RFM69_RST,  EE_WR, LED, MOTOR DIR #5, #3, #2
        // PORTSetPinsDigitalOut(IOPORT_D, BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_13);  // RFM69_CS, RFM69_RST,  EE_WR, LED, MOTOR DIR #5, #3, #2
        PORTSetPinsDigitalIn(IOPORT_E, BIT_9 | BIT_5 | BIT_7 | BIT_8);  // ENCODER DIRECTION INPUTS 1-4
        PORTSetPinsDigitalOut(IOPORT_G, BIT_12);  // MOTOR DIR #1       
    #else    
        PORTSetPinsDigitalOut(IOPORT_A, BIT_5);  // MOTOR_DIR4     
        PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_2);  // SW1-3
        PORTSetPinsDigitalOut(IOPORT_B, BIT_5);  // RS485_ENABLE
        PORTSetPinsDigitalIn(IOPORT_C, BIT_13);  // SW4
        PORTSetPinsDigitalOut(IOPORT_C, BIT_1);  // TEST_OUT        
        PORTSetPinsDigitalOut(IOPORT_D, BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_13);  // EE_WR, LED, MOTOR DIR #3, #2
        PORTSetPinsDigitalIn(IOPORT_E, BIT_9 | BIT_5 | BIT_7 | BIT_8);  // ENCODER DIRECTION INPUTS 1-4
        PORTSetPinsDigitalOut(IOPORT_G, BIT_12);  // MOTOR DIR #1          
    #endif    
    // mCNOpen(CN_ON, CN1_ENABLE | CN2_ENABLE | CN3_ENABLE | CN4_ENABLE | CN14_ENABLE, CN1_PULLUP_ENABLE | CN2_PULLUP_ENABLE | CN3_PULLUP_ENABLE | CN4_PULLUP_ENABLE | CN14_PULLUP_ENABLE);
    mCNOpen(CN_ON, CN14_ENABLE, CN14_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);      
#endif        
    
    RFM69_CS_HIGH();
    RESET_HIGH();
    TEST_OUT = 0;

    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;    
    PR2 = 4000; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip   
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);                    
    
    // Set up HOST UART    
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    // UARTConfigure(HOSTuart, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualHOSTBaudRate = UARTSetDataRate(HOSTuart, SYS_FREQ, 921600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);    
     
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();   
}// END InitializeSystem() 

// C++ level interrupt handler for this instance
// RH_RF69 is unusual in Mthat it has several interrupt lines, and not a single, combined one.
// On Moteino, only one of the several interrupt lines (DI0) from the RH_RF69 is connnected to the processor.
// We use this to get PACKETSDENT and PAYLOADRADY interrupts.

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) 
{   
static unsigned short PreviousPORTDRead = 0;
BYTE RFM69InterruptDetected = false;
    
    PORTD_Read = 0x0000;
    PORTD_Read = PORTD & 0b0000000000100000;
    
    RFM69_IntFlag = true;
    
    if (PORTD_Read != PreviousPORTDRead)
    {
        RFM69_IntFlag = true;   
        RFM69InterruptCounter++;
        RFM69InterruptDetected = PORTD_Read;
        PreviousPORTDRead = PORTD_Read;
    }
    if (PORTD_Read == 0) RFM69_INT_State = 0;
    else RFM69_INT_State = 1;    
 
    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();
    
    
    if (RFM69InterruptDetected)
    {            
        // Get the interrupt cause
        uint8_t irqflags2 = spiRead(RH_RF69_REG_28_IRQFLAGS2);
        if (_mode == RHModeTx && (irqflags2 & RH_RF69_IRQFLAGS2_PACKETSENT))
        {
            // A transmitter message has been fully sent
            setModeIdle(); // Clears FIFO
            _txGood++;
            //	Serial.println("PACKETSENT");
        }
        // Must look for PAYLOADREADY, not CRCOK, since only PAYLOADREADY occurs _after_ AES decryption
        // has been done
        if (_mode == RHModeRx && (irqflags2 & RH_RF69_IRQFLAGS2_PAYLOADREADY))
        {
            // A complete message has been received with good CRC
            _lastRssi = -((int8_t)(spiRead(RH_RF69_REG_24_RSSIVALUE) >> 1));
            _lastPreambleTime = millis();

            setModeIdle();
            // Save it in our buffer
            RH_RF69readFifo();            
            //	Serial.println("PAYLOADREADY");
        }    
    }
    
}


void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
uint8_t inByte;
static unsigned short index = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));                 
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                inByte = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(HOSTuart)) 
        {
            inByte = (uint8_t)UARTGetDataByte(HOSTuart);
            if (index < RH_RF69_MAX_MESSAGE_LEN)
                HOSTRxBuffer[index++] = inByte;
            if (inByte == ETX)
            {
                HOSTRxLength = index;
                HOSTRxBufferFull = true;
                index = 0;
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));            
}
