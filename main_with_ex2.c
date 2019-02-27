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
 * RFM69HCW UHF Radio Module initialization function
 *
 * - frequency = 433, nodeID = 1, networkID = 0
 * - ToNodeID = not used, encrypt (key) = 0,
 */
void init_UHF(void){
    // INITIAL CHECK
    write_SPI(REG_SYNCVALUE1, 0xAA);
    while(read_SPI(REG_SYNCVALUE1) != 0xAA);

    write_SPI(REG_SYNCVALUE1, 0x55);
    while(read_SPI(REG_SYNCVALUE1) != 0x55);

    // REGISTERS CONFIGURATION
    write_SPI(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
    write_SPI(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00); // no shaping
    write_SPI(REG_BITRATEMSB, RF_BITRATEMSB_55555); // default: 4.8 KBPS
    write_SPI(REG_BITRATELSB, RF_BITRATELSB_55555);
    write_SPI(REG_FDEVMSB, RF_FDEVMSB_50000); // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    write_SPI(REG_FDEVLSB, RF_FDEVLSB_50000);
    write_SPI(REG_FRFMSB, RF_FRFMSB_433);
    write_SPI(REG_FRFMID, RF_FRFMID_433);
    write_SPI(REG_FRFLSB, RF_FRFLSB_433);

    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
    write_SPI(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2); // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    write_SPI(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // DIO0 is the only IRQ we're using
    write_SPI(REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF); // DIO5 ClkOut disable for power saving
    write_SPI(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN); // writing to this bit ensures that the FIFO & status flags are reset
    write_SPI(REG_RSSITHRESH, 220); // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    write_SPI(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0);
    write_SPI(REG_SYNCVALUE1, 0x2D);      // attempt to make this compatible with sync1 byte of RFM12B lib
    write_SPI(REG_SYNCVALUE2, 0); // NETWORK ID
    write_SPI(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF);
    write_SPI(REG_PAYLOADLENGTH, 66); // in variable length mode: the max frame size, not used in TX
    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    write_SPI(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE); // TX on FIFO not empty
    write_SPI(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF); // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    write_SPI(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0); // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0

    // ENCRYPTION OFF
    write_SPI(REG_OPMODE, (read_SPI(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
    write_SPI(REG_PACKETCONFIG2, (read_SPI(REG_PACKETCONFIG2) & 0xFE) | 0);

    // HIGH POWER MODE ON ********
    write_SPI(REG_OCP, RF_OCP_ON);
    write_SPI(REG_PALEVEL, (read_SPI(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages

    // IF RF_OCP_OFF: writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);

    // Set an interrupt pin the RX buffer, MAYBE *********
    while(((read_SPI(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00)); // wait for ModeReady
}

/*
 * RFM69HCW UHF Radio Module get frequency function
 *
 * - FSTEP = 61.03515625 == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)
 */
int get_frequency(void){
    return 61.03515625  * (((long) read_SPI(REG_FRFMSB) << 16) + ((long) read_SPI(REG_FRFMID) << 8) + read_SPI(REG_FRFLSB));
}

/*
 * RFM69HCW UHF Radio Module set frequency function
 *
 * - FSTEP = 61.03515625 == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)
 */
void set_frequency(long freqHz){
    write_SPI(REG_OPMODE, (read_SPI(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);

    freqHz /= 61.03515625 ; // divide down by FSTEP to get FRF
    write_SPI(REG_FRFMSB, ((long) freqHz) >> 16);
    write_SPI(REG_FRFMID, ((long) freqHz) >> 8);
    write_SPI(REG_FRFLSB, ((long) freqHz));

    write_SPI(REG_OPMODE, (read_SPI(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
}

/*
 * RFM69HCW UHF Radio Module get RSSI function
 *
 * - RSSI range typically 0 - 100 or negative integers
 */
int get_RSSI(){
    int rssi = 0;

    // RSSI trigger not needed if DAGC is in continuous mode
    write_SPI(REG_RSSICONFIG, RF_RSSI_START);
    while ((read_SPI(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready

    rssi = -read_SPI(REG_RSSIVALUE);
    rssi >>= 1;
    return rssi;
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

            init_UHF();

            for(ex2_count = 0; ex2_count < 35; ex2_count++){
                uhf_data += 0.25;

                set_frequency(uhf_data);

                if(get_frequency() == uhf_data){
                    ex2_data[ex2_count] = get_RSSI();
                }
            }

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

