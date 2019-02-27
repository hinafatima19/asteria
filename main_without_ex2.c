/*
 * Senior Design ThinSat
 *
 * MSP430 - FR2355
 */

#include <msp430.h>
#include "RFM69registers.h"

enum system_states{INIT, EX2, EX3, TRANS};
enum system_states STATE = INIT;

int ex2_data[35];
int ex3_data[35];

int ex2_count = 0;
long uhf_data = 432.75;

int adc_count = 11;
int dac_count = 0;
int ex3_count = 0;
int dac_data = 0;

int trans_count = 0;

int temp = 0;

/*
 * External Oscillator initialization function
 *
 * X1 -----> 2.6 - X1
 * X2 -----> 2.7 - X2
 *
 * - 8Mhz external crystal
 * - 8Mhz / 8 = 1Mhz = MCLK = SMCLK
 */
void init_OSC(void){
    P2SEL1 |= (BIT6 | BIT7);
    P2SEL0 &= ~(BIT6 | BIT7);

    CSCTL6 |= XTS;
    CSCTL4 |= SELMS__XT1CLK | SELA__REFOCLK;

    do{
        CSCTL7 &= ~(XT1OFFG | DCOFFG);
        SFRIFG1 &= ~OFIFG;
    }
    while(SFRIFG1 & OFIFG);

    CSCTL5 |= DIVM_3;
}

/*
 * External Watch-dog initialization function
 *
 * WDT -----> 3.6 - GPIO OUTPUT, TB1
 *
 * - Ping before every 200ms ~ 150ms safe zone
 * - 1Mhz / 4 = 250khz * 150ms = 37500 < 65536 (2^16 bit timer)
 */
void init_WDT(void){
    P3OUT &= ~BIT6;
    P3DIR |= BIT6;

    TB1CCR0 = 37500;
    TB1CTL |= TBSSEL__SMCLK | ID_2 | MC_1 | TBCLR;
    TB1CCTL0 |= OUTMOD_7 | CCIE;
}

/*
 * UART initialization function
 *
 * TX   -----> 1.7 - USCI_A0
 * RX   -----> 1.6 - USCI_A0
 * GND  -----> GND
 * BUSY -----> 2.0 - GPIO INPUT
 *
 * - 4800 Baud Rate with Over-sampling (from this link:)
 * - http://processors.wiki.ti.com/index.php/USCI_UART_Baud_Rate_Gen_Mode_Selection
 */
void init_UART(void){
    P2DIR &= ~BIT0;
    P2REN |= BIT0;
    P2OUT |= BIT0;

    P1SEL0 |= (BIT6 | BIT7);
    P1SEL1 &= ~(BIT6 | BIT7);

    UCA0CTLW0 |= UCSWRST;

    UCA0CTLW0 |= UCSSEL__SMCLK;
    UCA0BR0 = 208;
    UCA0BR1 = 0;
    UCA0MCTLW |= 0x300;

    UCA0CTLW0 &= ~UCSWRST;
}

/*
 * SPI initialization function
 *
 * MOSI  -----> 4.3 - USCI_A1
 * MISO  -----> 4.2 - USCI_A1
 * SCLK  -----> 4.1 - USCI_A1
 * CS    -----> 4.0 - GPIO OUTPUT
 * GND   -----> GND
 * RF    -----> 1.2 - GPIO OUTPUT
 * DIO_0 -----> 6.5 - GPIO OUTPUT
 * DIO_1 -----> 6.4 - GPIO OUTPUT
 * DIO_2 -----> 6.3 - GPIO OUTPUT
 * DIO_3 -----> 6.2 - GPIO OUTPUT
 * DIO_4 -----> 6.1 - GPIO OUTPUT
 * DIO_5 -----> 6.0 - GPIO OUTPUT
 *
 * - Clock = SMCLK/60, /2
 * - UCCKPL?, 3-pin SPI, clock polarity high, MSB
 */
void init_SPI(void){
    P4OUT |= BIT0;
    P4DIR |= BIT0;

    P4SEL0 |= (BIT3 | BIT2 | BIT1);
    P4SEL1 &= ~(BIT3 | BIT2 | BIT1);

    P1DIR &= ~BIT2;
    P1DIR |= BIT2;

    P6OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
    P6DIR |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);

    UCA1CTLW0 |= UCSWRST;

    UCA1CTLW0 |= UCCKPH | UCMSB | UCMST | UCSYNC;
    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BR0 = 0x02;
    UCA1BR1 = 0;
    UCA0MCTLW = 0; // MAYBE NOT NEEDED

    UCA1CTLW0 &= ~UCSWRST;
}

/*
 * ADC initialization function
 *
 * MPPT_V -----> 5.0 - A8
 * DET_V  -----> 5.1 - A9
 * DET_I  -----> 5.2 - A10
 * MPPT_I -----> 5.3 - A11
 * LIGHT  -----> 1.3 - A3
 *
 * - 12-bit conversion with 16 Sample & Hold time
 * - VRef+ = AVCC = DVCC, VREF- = AVSS = DVSS
 * - SMCLK = 1Mhz / 4 = 250Khz
 */
void init_ADC(void){
    P1SEL0 |= BIT3;
    P1SEL1 |= BIT3;

    P5SEL0 |= (BIT0 | BIT1 | BIT2 | BIT3);
    P5SEL1 |= (BIT0 | BIT1 | BIT2 | BIT3);

    ADCCTL0 &= ~ADCENC;

    ADCCTL0 |= ADCSHT_2 | ADCMSC | ADCON;
    ADCCTL1 |= ADCSHP | ADCSSEL_2 | ADCDIV_3 | ADCCONSEQ_1;
    ADCCTL2 &= ~ADCRES;
    ADCCTL2 |= ADCRES_2;
    ADCIE |= ADCIE0;

    ADCMCTL0 |= ADCINCH_11;
}

/*
 * DAC initialization function
 *
 * L_Set -----> 3.5 - OA3O
 *
 * - 12-bit conversion with 12-bit dac_data
 * - DACSREF = 0 = DVCC, DACLSEL = 2 = write to SAC3DACDAT register
 * - PSEL_1 = DAC, PSEL_2 = PGA, Buffered Mode
 */
void init_DAC(void){
    P3SEL0 |= BIT5;
    P3SEL1 |= BIT5;

    SAC3DAC &= ~DACEN;
    SAC3OA &= ~(SACEN | OAEN);

    SAC3OA = NMUXEN | PMUXEN | PSEL_1 | NSEL_1;
    SAC3PGA = MSEL_1;
}

/*
 * UART Write function
 *
 * - HIGH = busy, LOW = not busy
 * - Max value for 8-bit data buffer = 255
 */
void write_UART(int value){
    while(P2IN & BIT0);
    UCA0TXBUF = value;
    while(!(UCA0IFG & UCTXIFG0));
}

/*
 * SPI Write function
 *
 * - Checks TX and RX flags
 * - Sends address first then data
 */
void write_SPI(int addr, int value){
    temp = 0;

    P4OUT &= ~BIT0;

    UCA1TXBUF = addr;
    while(!(UCA1IFG & UCTXIFG));

    while(!(UCA1IFG & UCRXIFG));
    temp = UCA1RXBUF;

    UCA1TXBUF = value;
    while(!(UCA1IFG & UCTXIFG));

    while(!(UCA1IFG & UCRXIFG));
    temp = UCA1RXBUF;

    P4OUT |= BIT0;
}

/*
 * SPI Read function
 *
 * - Checks TX and RX flags
 * - Sends address with read command first then blank data
 */
int read_SPI(int addr){
    temp = 0;

    P4OUT &= ~BIT0;

    UCA1TXBUF = addr | 0x80;
    while(!(UCA1IFG & UCTXIFG));

    while(!(UCA1IFG & UCRXIFG));
    temp = UCA1RXBUF;

    UCA1TXBUF = 0x00;
    while(!(UCA1IFG & UCTXIFG));

    while(!(UCA1IFG & UCRXIFG));
    temp = UCA1RXBUF;

    P4OUT |= BIT0;

    return temp;
}

/*
 * WDT timer ISR
 *
 * - Triggered when 150ms reached
 */
#pragma vector = TIMER1_B0_VECTOR
__interrupt void WDT_ISR (void){
    P3OUT |= BIT6;
    P3OUT &= ~BIT6;
}

/*
 * ADC flag ISR
 *
 * - Triggered when ADC completes
 */
#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR (void){
    switch(__even_in_range(ADCIV, ADCIV_ADCIFG)){
        case ADCIV_ADCIFG:
            if(adc_count == 11){
                ex3_data[ex3_count] = (ADCMEM0 / (4095 / 255));
                ex3_count++;
            }
            else if(adc_count == 10){
                ex3_data[ex3_count] = (ADCMEM0 / (4095 / 255));
                ex3_count++;
            }
            else if(adc_count == 9){
                ex3_data[ex3_count] = (ADCMEM0 / (4095 / 255));
                ex3_count++;
            }
            else if(adc_count == 8){
                ex3_data[ex3_count] = (ADCMEM0 / (4095 / 255));
                ex3_count++;
            }
            else if(adc_count == 3){
                ex3_data[ex3_count] = (ADCMEM0 / (4095 / 255));
                ex3_count++;
            }
            else{
                temp = ADCMEM0;
                temp = 0;
            }

            adc_count--;
            break;
        default: break;
    }
}

/*
 * main function
 *
 */
void main(void){
    WDTCTL = WDTPW | WDTHOLD;       // Stop internal watch-dog timer

    if(STATE == INIT){
        init_OSC();
        init_WDT();
        init_UART();
        init_SPI();
        init_ADC();
        init_DAC();

        PM5CTL0 &= ~LOCKLPM5;       // Unlock ports from power manager

        __enable_interrupt();       // Enable interrupts

        STATE = EX2;
    }
    while(1){
        if(STATE == EX2){
            ex2_count = 0;

            P1OUT |= BIT2;

            //radio.initialize
            //radio.setHighPower
            //for{
            //  radio.setFrequency
            //  radio.readRSSI
            //  store in array
            //}

            P1OUT &= ~BIT2;

            STATE = EX3;
        }
        else if(STATE == EX3){
            ex3_count = 0;
            dac_count = 0;
            dac_data = 0;

            for(dac_count = 0; dac_count < 7; dac_count++){
                adc_count = 11;

                SAC3DAC |= DACEN;                       // Enable DAC
                SAC3OA |= SACEN | OAEN;                 // Enable DAC Output
                SAC3DAT = dac_data + 585;

                while(adc_count >= 3){
                    ADCCTL0 |= ADCENC | ADCSC;          // Enable ADC and Start
                    while(ADCCTL1 & ADCBUSY);
                }

                ADCCTL0 &= ~ADCENC;                     // Disable ADC
            }

            SAC3DAC &= ~DACEN;                          // Disable DAC
            SAC3OA &= ~(SACEN | OAEN);                  // Disable DAC Output

            STATE = TRANS;
        }
        else if(STATE == TRANS){
            trans_count = 0;

            for(trans_count = 0; trans_count < 35; trans_count++){      // Write Experiment 2 Data
                write_UART(ex2_data[trans_count]);
            }

            trans_count = 0;

            for(trans_count = 0; trans_count < 35; trans_count++){      // Write Experiment 3 Data
                write_UART(ex3_data[trans_count]);
            }

            STATE = EX2;
        }
    }
}

