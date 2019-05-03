/*
 * Senior Design ThinSat Phase 2
 *
 *
 *********************************** SYSTEM FLOW **********************************
 * - System states include initialize, experiment 2, experiment 3, and transmit
 *
 *        [POWER ON]   ->   INIT   ->   EX2   ->   EX3   ->   TRANS   ->   [POWER OFF]
 *                                       ^                       |
 *                                       |                       |
 *                                       '-- [POWER STILL ON] <--'
 *
 *
 *********************************** PACKET FORMAT **********************************
 * - 35 byte long packets , excluding 3 header bytes per 35 byte packet , total size is 38 bytes
 *
 *                 0              1              2              3              4          ....      33                  34
 * EXP2:     | FRQ = 433.00 | FRQ = 433.25 | FRQ = 433.50 | FRQ = 433.75 | FRQ = 434.00 | .... | FRQ = 441.25 | Experiment #: 2 |
 *
 *                 0              1              2              3              4          ....      33                  34
 * EXP3:     | DAC = 585    | DAC = 585    | DAC = 585    | DAC = 585    | DAC = 585    | .... | DAC = 4095   | Experiment #: 3 |
 *           | ADC = MPPT_I | ADC = DET_I  | ADC = DET_V  | ADC = MPPT_V | ADC = LIGHT  | .... | ADC = MPPT_V |                 |
 *
 *
 *********************************** DAC VALUES CALCULATION **********************************
 * - 3.3v / 150 resistor = 22mA , 22mA / 7 = 3.143mA steps , 585 DAC value steps (0 - 4095 range)
 * - Scaling: original / (4095 / 255) = scaled -> scaled * (4095 / 255) = original
 *
 *           DAC         I         R            V          DAC / VRef      DAC Value
 * EXP3:      1   |   3.143   *   150   =   0.47145   *   4096 / 3.3   =   585.16945
 *            2   |   6.286   *   150   =   0.9429    *   4096 / 3.3   =   1170.338909
 *            3   |   9.429   *   150   =   1.41435   *   4096 / 3.3   =   1755.50836
 *            4   |  12.572   *   150   =   1.8858    *   4096 / 3.3   =   2340.67781
 *            5   |  15.715   *   150   =   2.35725   *   4096 / 3.3   =   2925.8472
 *            6   |  18.858   *   150   =   2.8287    *   4096 / 3.3   =   3511.01672
 *            7   |  22.001   *   150   =   3.30015   *   4096 / 3.3   =   4096.18618 ( = 4095)
 *
 *
 *********************************** TIMER CALCULATION **********************************
 * - Blink every 15 seconds , 1 / 15sec = 0.666 Hz , 32768 / 8 = 4096 * 15 , SMCLK = 1MHz , 1 / 4096 = 0.2mSec , 148ms / 20ms
 *
 *          OSC        DIV        SPEED        PING        TICKS          MAX TIMER TICKS (2^16)
 * CLOCK:   1MHz   /   4     =   250kHz   *   150ms   =   37,500    <    65,536
 *          8MHz   /   4     =   2kHz     *   100ms   =   200,000   !<   65,536
 *          8MHz   /   8/2   =   500kHz   *   100ms   =   50,000    <    65,536
 *
 *
 *********************************** UART BAUD RATE CALCULATION **********************************
 * - Baud rate dependent on the MCLK speed not external oscillator
 *
 * 1MHz 38400:  UCA0BRW =                       UCBRx
 *                              | 15 | 14 | 13 | 12 | 11 | 10 | 9 | 8 |
 *                                              UCBRx
 *                              | 7  | 6  | 5  | 4  | 3  | 2  | 1 | 0 |
 *
 *              UCBRx = 1 = 0000 0000 0000 0001
 *              UCA0BR0 = 1 (Lower 8 bits)
 *              UCA0BR1 = 0 (Upper 8 bits)
 *              UCA0BRW = 1
 *
 *              UCA0MCTLW =                     UCBRSx
 *                              | 15 | 14 | 13 | 12 | 11 | 10 | 9 | 8 |
 *                                  UCBRFx                 NA    UCOS16
 *                              | 7  | 6  | 5  | 4  | 3  | 2  | 1 | 0 |
 *
 *              UCA0BRFx = 10 = 0000 0000 0000 1010
 *              UCA0BRSx = 0x00 = 0000 0000 0000 0000
 *              UCOS16 = 1 = 0000 0000 0000 0001
 *              UCA0MCTLW = 0000 0000 1010 0001 = 0xA1 = 161
 *
 *
 */

#include <msp430.h>
#include "RFM69registers.h"

enum system_states{INIT, EX2, EX3, TRANS};
enum system_states STATE = INIT;

int ex2_data[35];
int ex3_data[35];

int ex2_count = 0;
long uhf_data = 432750000;
long const freq_val = 61.03515625;

int ex3_count = 0;
int adc_count = 11;
int dac_count = 0;
int dac_data = 0;

int trans_count = 0;
int exp_count = 0;
int recv_data = 0;

int temp = 0;

/*
 * External Oscillator initialization function
 *
 * X1 -----> 2.6 - X1
 * X2 -----> 2.7 - X2
 *
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
 * UART initialization function
 *
 * TX   -----> 1.7 - USCI_A0
 * RX   -----> 1.6 - USCI_A0
 * BUSY -----> 2.0 - GPIO INPUT
 * GND  -----> GND
 *
 * - 1Mhz SMCLK, 38400 Baud Rate (Dividers from data sheet)
 */
void init_UART(void){
    P2DIR &= ~BIT0;
    P2REN |= BIT0;
    P2OUT |= BIT0;

    P1SEL0 |= (BIT6 | BIT7);
    P1SEL1 &= ~(BIT6 | BIT7);

    UCA0CTLW0 |= UCSWRST;

    UCA0CTLW0 |= UCSSEL__SMCLK;
    UCA0BRW = 1;
    UCA0MCTLW = 161;

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
 *
 * - 1Mhz / (BRW = 2) = SMCLK, 38400 SPI Speed
 */
void init_SPI(void){
    P4OUT |= BIT0;
    P4DIR |= BIT0;

    P4SEL0 |= (BIT3 | BIT2 | BIT1);
    P4SEL1 &= ~(BIT3 | BIT2 | BIT1);

    UCA1CTLW0 |= UCSWRST;

    UCA1CTLW0 |= UCCKPH | UCMSB | UCMST | UCSYNC;
    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BRW = 2;
    UCA1MCTLW = 0;

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
 * - VRef+ = AVCC = DVCC, VREF- = AVSS = DVSS
 * - 1Mhz SMCLK / 4 = 250Khz SMCLK
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
 * - DACSREF = 0 = DVCC
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
 * - Max buffer value is 2^8 which is 255
 */
void write_UART(int value){
    UCA0TXBUF = value;
    while(!(UCA0IFG & UCTXIFG0));
}

/*
 * SPI Read function
 *
 * - Sends address first then zeros to read the register value
 */
int read_SPI(int addr){
    temp = 0;

    P4OUT &= ~BIT0;

    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = addr & 0x7F;
    while(!(UCA1IFG & UCRXIFG));
    temp = UCA1RXBUF;

    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = 0x00;
    while(!(UCA1IFG & UCRXIFG));
    temp = UCA1RXBUF;

    P4OUT |= BIT0;

    return temp;
}

/*
 * SPI Write function
 *
 * - Sends address first then data
 */
int write_SPI(int addr, int value){
    temp = 0;

    P4OUT &= ~BIT0;

    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = addr | 0x80;
    while(!(UCA1IFG & UCRXIFG));
    temp = UCA1RXBUF;

    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = value;
    while(!(UCA1IFG & UCRXIFG));
    temp = UCA1RXBUF;

    P4OUT |= BIT0;

    return temp;
}

/*
 * RFM69HCW UHF Radio Module initialization function
 */
void init_UHF(void){
    while(read_SPI(REG_SYNCVALUE1) != 0xAA){
        write_SPI(REG_SYNCVALUE1, 0xAA);
    }

    while(read_SPI(REG_SYNCVALUE1) != 0x55){
        write_SPI(REG_SYNCVALUE1, 0x55);
    }

    write_SPI(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
    write_SPI(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
    write_SPI(REG_BITRATEMSB, RF_BITRATEMSB_55555);
    write_SPI(REG_BITRATELSB, RF_BITRATELSB_55555);
    write_SPI(REG_FDEVMSB, RF_FDEVMSB_50000);
    write_SPI(REG_FDEVLSB, RF_FDEVLSB_50000);
    write_SPI(REG_FRFMSB, RF_FRFMSB_433);
    write_SPI(REG_FRFMID, RF_FRFMID_433);
    write_SPI(REG_FRFLSB, RF_FRFLSB_433);
    write_SPI(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2);
    write_SPI(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
    write_SPI(REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF);
    write_SPI(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
    write_SPI(REG_RSSITHRESH, 220);
    write_SPI(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0);
    write_SPI(REG_SYNCVALUE1, 0x2D);
    write_SPI(REG_SYNCVALUE2, 0);
    write_SPI(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF);
    write_SPI(REG_PAYLOADLENGTH, 66);
    write_SPI(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
    write_SPI(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
    write_SPI(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);
    write_SPI(REG_OPMODE, (read_SPI(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
    write_SPI(REG_PACKETCONFIG2, (read_SPI(REG_PACKETCONFIG2) & 0xFE) | 0);
    write_SPI(REG_OCP, RF_OCP_ON);
    write_SPI(REG_PALEVEL, (read_SPI(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON);

    while(((read_SPI(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00));
}

/*
 * RFM69HCW UHF Radio Module get frequency function
 */
long get_frequency(void){
    return freq_val * (((long) read_SPI(REG_FRFMSB) << 16) + ((long) read_SPI(REG_FRFMID) << 8) + read_SPI(REG_FRFLSB));
}

/*
 * RFM69HCW UHF Radio Module set frequency function
 */
void set_frequency(long freqHz){
    freqHz /= freq_val;

    write_SPI(REG_OPMODE, (read_SPI(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
    write_SPI(REG_FRFMSB, (((long) freqHz) >> 16) & 0xff);
    write_SPI(REG_FRFMID, (((long) freqHz) >> 8) & 0xff);
    write_SPI(REG_FRFLSB, ((long) freqHz) & 0xff);
    write_SPI(REG_OPMODE, (read_SPI(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
}

/*
 * RFM69HCW UHF Radio Module get RSSI function
 */
int get_RSSI(){
    int rssi = 0;

    write_SPI(REG_RSSICONFIG, RF_RSSI_START);

    rssi = -read_SPI(REG_RSSIVALUE);
    rssi >>= 1;

    return rssi;
}

/*
 * ADC flag ISR
 */
#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR (void){
    switch(__even_in_range(ADCIV, ADCIV_ADCIFG)){
        case ADCIV_ADCIFG:
            if(adc_count == 11){
                ex3_data[ex3_count] = (ADCMEM0 / (4095 / 255));     // MPPT_I
                ex3_count++;
            }
            else if(adc_count == 10){
                ex3_data[ex3_count] = (ADCMEM0 / (4095 / 255));     // DET_I
                ex3_count++;
            }
            else if(adc_count == 9){
                ex3_data[ex3_count] = (ADCMEM0 / (4095 / 255));     // DET_V
                ex3_count++;
            }
            else if(adc_count == 8){
                ex3_data[ex3_count] = (ADCMEM0 / (4095 / 255));     // MPPT_V
                ex3_count++;
            }
            else if(adc_count == 3){
                ex3_data[ex3_count] = (ADCMEM0 / (4095 / 255));     // LIGHT
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
 */
void main(void){
    WDTCTL = WDTPW | WDTHOLD;   // Stop internal watch-dog timer

    PM5CTL0 &= ~LOCKLPM5;       // Unlock ports from power manager

    if(STATE == INIT){
        init_OSC();
        init_UART();
        init_SPI();
        init_ADC();
        init_DAC();

        __enable_interrupt();   // Enable interrupts

        STATE = EX2;
    }
    while(1){
        if(STATE == EX2){
            ex2_count = 0;
            uhf_data = 432750000;

            init_UHF();

            for(ex2_count = 0; ex2_count < 35; ex2_count++){
                uhf_data += 250000;

                write_SPI(REG_OPMODE, (read_SPI(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);

                set_frequency(uhf_data);

                __delay_cycles(100000);

                write_SPI(REG_RSSITHRESH, 0xFF);

                write_SPI(REG_OPMODE, (read_SPI(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);

                long t = (uhf_data - get_frequency());

                while(t >= 100){
                    write_SPI(REG_OPMODE, (read_SPI(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);

                    set_frequency(uhf_data);

                    __delay_cycles(100000);

                    write_SPI(REG_RSSITHRESH, 0xFF);

                    write_SPI(REG_OPMODE, (read_SPI(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
                }

                ex2_data[ex2_count] = abs(get_RSSI());
            }

            STATE = EX3;
        }
        else if(STATE == EX3){
            ex3_count = 0;
            dac_count = 0;
            dac_data = 0;

            for(dac_count = 0; dac_count < 7; dac_count++){
                adc_count = 11;

                SAC3DAC |= DACEN;               // Enable DAC
                SAC3OA |= SACEN | OAEN;         // Enable DAC Output

                dac_data += 585;
                SAC3DAT = dac_data;

                __delay_cycles(100000);

                while(adc_count >= 3){
                    ADCCTL0 |= ADCENC | ADCSC;  // Enable ADC and Start

                    while(ADCCTL1 & ADCBUSY);

                    __delay_cycles(100000);

                }

                ADCCTL0 &= ~ADCENC;             // Disable ADC
            }

            SAC3DAC &= ~DACEN;                  // Disable DAC
            SAC3OA &= ~(SACEN | OAEN);          // Disable DAC Output

            STATE = TRANS;
        }
        else if(STATE == TRANS){
            trans_count = 0;
            exp_count = 0;
            recv_data = 0;

            ex2_data[34] = 2;
            ex3_data[34] = 3;

            for(exp_count = 0; exp_count < 2; exp_count++){
                while(P2IN & BIT0);                                         // OBC is NOT BUSY, Send OBC our data

                for(trans_count = 0; trans_count < 3; trans_count++){
                    write_UART(80);                                         // Write packet header
                }

                if(exp_count == 0){
                    for(trans_count = 0; trans_count < 35; trans_count++){
                        write_UART(ex2_data[trans_count]);                  // Write Experiment 2 Data
                    }
                }
                else if(exp_count == 1){
                    for(trans_count = 0; trans_count < 35; trans_count++){
                        write_UART(ex3_data[trans_count]);                  // Write Experiment 3 Data
                    }
                }

                __delay_cycles(100000);
            }

            STATE = EX2;
        }
    }
}
