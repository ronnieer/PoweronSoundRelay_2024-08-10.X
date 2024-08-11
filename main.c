/*
 * File:   main.c
 * Author: Ronnie
 *
 * Created on den 30 januari 2024, 12:42
 */


// PIC16F1826 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
//#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config WDTE = OFF    // Watchdog Timer Enable->WDT disabled
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config DEBUG = OFF      // In-Circuit Debugger Mode (In-Circuit Debugger disabled, ICSPCLK and ICSPDAT are general purpose I/O pins)
#pragma config LVP = OFF         // Low-Voltage Programming Enable (Low-voltage programming enabled)




// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic16f1826.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ 31000                        // Internal Oacillator is set to 31000 Hz (Resonator is 4.00 Mhz)

uint16_t mainsPeriodCounter = 0;

void portsSetup(void);
void OSCILLATOR_Initialize(void);
void interruptSetup(void);
void TMR1_Initialize(void);
void TMR1_StartTimer(void);
void run(void);

void OSCILLATOR_Initialize(void){
    OSCCON = 0x02;                              // SCS INTOSC; SPLLEN disabled; IRCF 31KHz_LF; 
    OSCTUNE = 0x00;                             // TUN 0;  
    BORCON = 0x00;                              // SBOREN disabled;
}

void interruptSetup(void){
    INTCONbits.GIE = true;                      // Main Interrupt Enabled
    INTCONbits.PEIE = true;                     // Peripheral Interrupt Enable
    PIE1bits.TMR1IE = true;                     // Timer 1 Interrupt Enabled
    PIE1bits.TMR2IE = true;                     // Timer 2 Interrupt Enabled
}

void TMR1_Initialize(void){
    T1GCON = 0x00;
    TMR1H = 0xB4;                               // TMR1 0xB450, T1CON 0x35 => 20 seconds
    TMR1L = 0x50;
    PIR1bits.TMR1IF = false;                    // Timer 1 Interruptflag
    T1CON = 0x35;
}

void TMR1_StartTimer(void){
    T1CONbits.TMR1ON = true;                    // Timer 1 On
}

void TMR2_Initialize(void){
    T2CONbits.T2OUTPS = 0x01;                   // Timer 2 set to ~ 52 milliseconds, which is the dropout period for mains power    
    PR2 = 200;  
}

void TMR2_StartTimer(void){
    T2CONbits.TMR2ON = true;                    // Timer 2 On
}

void portsSetup(void){
    ANSELA = 0x00;                              // Make Port A None Analogue
    TRISA = 0x02;                               // Make Port A bit One input, the rest of the Port pins are Outputs
    TRISB = 0x00;
}

void main(void) {
    portsSetup();                               // Set Ports Functionality
    OSCILLATOR_Initialize();                    // Set Oscillator Functionality
    interruptSetup();                           // Set Interrupt Functionality
    TMR1_Initialize();              
    TMR2_Initialize();              
    TMR1_StartTimer();
    TMR2_StartTimer();    
    run();                                      // Run Program
}

void __interrupt()Isr(void){   
    if(PIR1bits.TMR2IF){                        // Timer 2 Interruptflagg. Interrupt 2 Flagg is set when the uC detects missing Mains AC supply
        PORTAbits.RA0 = 0;                      // Sound Output Relay Off
        TMR1H = 0xB4;                           // Set Time to 20 seconds. This is acheaved combined with OSC/4 and prescaler set to 8
        TMR1L = 0x50;
        PIR1bits.TMR1IF = false;                // Clear the timers interruptflaggs
        PIR1bits.TMR2IF = false;
    }
    if(PIR1bits.TMR1IF){                        // If Timer 1 lasts 20 seconds with no reset (no time out at Timer 2 (52 milliseconds))
        PORTAbits.RA0 = 1;                      // Output Sound Relay goes on
    }   
    PIR1 = 0x00;                                // Prepare ISR exit by clearing all interruptflaggs          
    PIR2 = 0x00;    
}

void run(void){
    PORTAbits.RA0 = 0;                          // Start main routine by clearing Output Relay
    while(true){
        if(PORTAbits.RA1) TMR2_Initialize();    // if AC power is on timer 2 is periodically reset at 100 Hz, making Timer 2 reset for 52 new milliseconds
    }
}
