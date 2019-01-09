#include <msp430.h>

unsigned int send[8] = {1,2,3,4,5,6,7,8};

void initMSP430(void){
    WDTCTL = WDTPW | WDTHOLD;   // Stop the watchdog

    P2SEL0 |= BIT0 | BIT1;      // TX, RX
    P2SEL1 &= ~(BIT0 | BIT1);

    UCA0CTLW0 |= UCSWRST;       // Reset UCA0
    UCA0CTLW0 |= UCSSEL_3;      // SMCLK
    UCA0BR0 = 208;              // BAUD CALCULATED FROM MSP430 WIKI:
    UCA0BR1 = 0;                // http://processors.wiki.ti.com/index.php/USCI_UART_Baud_Rate_Gen_Mode_Selection
    UCA0MCTLW |= 0x300;

    UCA0CTLW0 &= ~UCSWRST;      // Release from Reset

    PM5CTL0 &= ~LOCKLPM5;       // Unlock ports from power manager

    __enable_interrupt();       // Enable Interrupts
}

#pragma vector = PORT4_VECTOR
__interrupt void Busy_ISR(){
    switch (P4IV){
        case P4IV_P4IFG0:       // Busy signal is 1
            break;
        default: break;
    }
}

void main(void){
    initMSP430();
    int i = 0;

    while(1){
        if(!(P4IN & BIT0)){     // Busy signal is 0
            UCA0TXBUF = 255;    // MAX VALUE FOR 8-bit DATA BUFFER
            while(!(UCA0IFG & UCTXIFG0));
        }
    }
}

