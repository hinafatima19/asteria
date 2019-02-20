#include <msp430.h> 
#include "driverlib.h"
#include "Board.h"
/**
 * main.c
 */

enum packet_seq {HEADER,DATA}
enum exp_status {EXP_2, EXP_3,TRANS}

int mpptValues[35]; // MPPT Packet
int detValues[35]; // DET Packet
int lightTempValues[35]; //
int uhfValues[8][35]; // UHF Packet
unsigned int DAC_data = 0;
enum exp_status exp;
enum packet_seq packet = HEADER;
int exp3_tracker; // refers to a location in either the MPPT,DET of light/temp packet
int adcseqnum;
int measurement = 0;

/*
 *  To be completed: halp needed
 *  -- SPI read from UHF chip onboard Payload
 *  -- insert each byte into array containing experiment 2 data
 */
void UHFRead(void) {

}

void run_exp2(void){

}

/**
 * function to run experiment 3
 */
void run_exp3(int pos){


    int flag = 0;
    if (pos > 17){
        measurement = 45;
        return;
    }
    while (1){
        if(adcseqnum == 2){
            ADCCCTL0 &= (ADCENC); // turn off ADC
            break;
        }
        else{
            while(ADCCTL1 & ADCBUSY); // Wait if ADC core is active
            flag = 1;
            ADCCTL0 |= ADCENC | ADCSC;
            while(flag);
        }
    }
    measurement++;
}


void Software_Trim(void) // Credit to Darren Lu of Texas Instruments
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100;                         // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;                  // Clear DCO fault flag
        }while (CSCTL7 & DCOFFG);               // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ);// Wait FLL lock status (FLLUNLOCK) to be stable
                                                           // Suggest to wait 24 cycles of divided FLL reference clock
        while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;                   // Read CSCTL0
        csCtl1Read = CSCTL1;                   // Read CSCTL1

        oldDcoTap = newDcoTap;                 // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;       // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070)>>4;// Get DCOFTRIM value

        if(newDcoTap < 256)                    // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;     // Delta value between DCPTAP and 256
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256)) // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }
        else                                   // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;     // Delta value between DCPTAP and 256
            if(oldDcoTap < 256)                // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }

        if(newDcoDelta < bestDcoDelta)         // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    }while(endLoop == 0);                      // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy;                       // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy;                       // Reload locked DCOFTRIM
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}

void setupLXT(void){ // Credit to Darren Lu of Texas Instruments

    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

    P2SEL1 |= BIT6 | BIT7;                  // P2.6~P2.7: crystal pins

    do
      {
        CSCTL7 &= ~(XT1OFFG | DCOFFG);      // Clear XT1 and DCO fault flag
         SFRIFG1 &= ~OFIFG;
     } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag

     __bis_SR_register(SCG0);                // disable FLL
     CSCTL3 |= SELREF__XT1CLK;               // Set XT1CLK as FLL reference source
     CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3;// DCOFTRIM=3, DCO Range = 8MHz
     CSCTL2 = FLLD_0 + 243;                  // DCODIV = 8MHz
     __delay_cycles(3);
     __bic_SR_register(SCG0);                // enable FLL
     Software_Trim();                        // Software Trim to get the best DCOFTRIM value

     CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;  // Set ACLK = XT1CLK = 32768Hz
                                                // DCOCLK = MCLK and SMCLK source
      CSCTL5 |= DIVM_0 | DIVS_1;
}

void setupUART(void){ // initialize UART module


    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_UCA0TXD, // configure TX pin
                                               GPIO_PIN_UCA0TXD,
                                               GPIO_FUNCTION_UCA0TXD);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_UCA0RXD, // configure RX pin
                                               GPIO_PIN_UCA0RXD,
                                               GPIO_FUNCTION_UCA0RXD);

    // Baud rate and other parameters taken from table of recommended settings
    // Here our BRCLK = ACLK so
    // frBRCLK = 32.8 kHZ
    // Baudrate = 4800
    // N = 32768 / 4800 = 6
    // UCBRx = 6
    // UCBRFx = ignored since UCOS16 = 0
    // UCBRSx = 0x20
    UCA0CTLW0 = UCSWRST;                      // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__ACLK;               // CLK = ACLK
    UCA0BRW = 6;
    UCA0MCTLW = 0xEE00;
    UCA0CTLW0 &= ~UCSWRST;

}

void setupWatchDog(void){ // ping

    TB1CCR0 = 12500; // 0.150 second period
    TB1EX0 = TAIDEX_3;
    TB1CCTL1 = OUTMOD_7;
    TB1CTL = TBSSEL_2 | ID_2 | MC_1 | TBCLR; // Divide SMCLK by 12
    TB1CCTL0 = CCIE;
    P3DIR |=  BIT6; // WDT pin = output pin
    P3OUT ^= BIT6;

}

void setupSPI(void){ // setup SPI master

    //Configure STE and CLK pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                                               GPIO_PIN0 + GPIO_PIN1,
                                               GPIO_SECONDARY_MODULE_FUNCTION);
    //Configure SIMO, SOMI
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
                                               GPIO_PIN2 + GPIO_PIN3,
                                               GPIO_TERNARY_MODULE_FUNCTION);

   // Configure SPI module
   UCA1CTLW0 = UCSWRST;
   UCA1CTLW0 = UCMSB | UCSYNC | UCMST | UCSSEL__SMCLK;
   UCA1BR0 = 208; // Baud rate 4800
   UCA1BR1 = 0;
   UCACTLW0 &= ~(UCSWRST);

}



/*
 * Initialized the ADC using the following setup properties:
 *
 * Analog Input Pins:
 * (1) Pin P5.0 -> Channel A8 (MMPT_VOUT)
 * (2) Pin P5.1 -> Channel A9 (DET_VOUT)
 * (3) Pin P5.2 -> Channel A10 (DET_I)
 * (4) Pin P5.3 -> Channel A11 (MMPT_I)
 * (5) Pin P1.3 -> Channel A3
 *
 * Sequence of Channel Measurement
 * ADCCLK = ACLK/8 = 32768 / 4 = 8192 Hz
 * Sample/Hold Time = 16 * 1/8192 = 1.95 ms
 * Conversion Time = 15 * (1/8192) = 1.83 ms
 * Total Time between sampling and conversion of 1 ADC input = 1.95 ms + 1.83 ms = 3.78 or about 4 ms
 * Software Trigger used to start conversion
 * Internal 2.5V reference used.
 * Extended Sample Mode used.
 * Sequence of channel mode used. The ADC will sample channels A11-A8 and then ignore channels A7-A4 before sampling channel A3
 */
void setupADC(void){

    adcseqnum = 11;
    exp3_tracker = 0;
    P1SEL0 |= BIT3;
    P1SEL1 |= BIT3; // Light Sensor

    P5SEL0 |= (BIT3 | BIT2 | BIT1 | BIT0); // MPPT/DET voltage/current readings
    P5SEL1 |= (BIT3 | BIT2 | BIT1 | BIT0);



    ADCCTL0 &= ~ADCENC; // Disable ADC12 to control configuration
    ADCCTL0 = ADCSHT2 | ADCON; // Sample Hold = 16*ACCLK
    //ADCCTL0 = ADCSHT2 | ADCMSC | ADCON; // Sample Hold = 16*ACCLK, multi0sample on completion of prior
    ADCCTL1 = ADCCONSEQ_1 | ADCDIV_3 | ADCSSEL_1; // EXTENDED SAMPLE MODE, divide ACLK by 4
    ADCCTL2 &= ~(ADCRES); // clear ADCRES in ADCCTL2
    ADCCTL2 |= ADCRES_2;
    ADCIE |= ADCIE0; // enable ADC conversion complete interrupt
    ADCMCTL0 |= ADCINCH_11 | ADCSREF_1 // enable internal 2.5 V reference

    // Configure reference
    PMMCTL0_H = PMMPW_H;                                // Unlock the PMM registers
    PMMCTL2 |= INTREFEN | REFVSEL_2;                    // Enable internal 2.5V reference
    __delay_cycles(400);

}

/*
 * Initialize the DAC using the following setup properties:
 * Using Pin P3.6 as OA output
 * OA: configured to buffer mode
 * PGA MSEL: set to floating status
 * Vref = internal 2.5 V (can connect to external reference voltage if need be)
 * DACLSEL = 00b : DAC output is updated immediately when data is written to the DACDAT register
 *
 */
void setupDAC(void){

  P3SEL0 |= BIT6; // selecting P1.1 as OA0O function
  P3SEL1 |= BIT6; // set as buffer for DAC
  DAC_data = 0;
  // Configure reference module
  PMMCTL0_H = PMMPW_H;                      // Unlock the PMM registers
  PMMCTL2 = INTREFEN | REFVSEL_2;           // Enable internal 2.5V reference
  while(!(PMMCTL2 & REFGENRDY));            // Poll till internal reference settles

  SAC3DAC = DACSREF_1;  // Select int Vref as DAC reference
  SAC3DAT = DAC_data;                       // Initial DAC data
  SAC3DAC |= DACEN;                         // Enable DAC

  SAC3OA = NMUXEN + PMUXEN + PSEL_1 + NSEL_1;//Select positive and negative pin input
  SAC3OA |= OAPM;                            // Select low speed and low power mode
  SAC3PGA = MSEL_1;                          // Set OA as buffer mode
  SAC3OA |= SACEN + OAEN;                    // Enable SAC and OA

}

void sendDataOut(int experiment, int frames){

   int count = 0;

   if(experiment == 0){

      while(count < frames){
         int i;
         for(i = 0; i < 35; i++){
           while(!(P2IN & BIT0)){} // wait for busy flag
           EUSCI_A_UART_transmitData(EUSCI_A0_BASE, uhfValues[count][i]); // transmit data
           while(!(EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, UCTXIFG))); // wait for transmit buffer to empty itself
         }
      }

   }

   else{

       int dataVal;

    while(count < frames){
        int i;
        for(i = 0; i < 35; i++){

            if(count == 0){
                dataVal = detValues[i]; // get DET value from buffer
            }
            else if(count == 1){
                dataVal = mpptValues[i]; // get MPPT value from buffer
            }

            else{
                dataVal = lightTempValues[i]; // get light/temperature value from buffer
            }

          while(!(P2IN & BIT3)){} // wait for busy flag
           EUSCI_A_UART_transmitData(EUSCI_A0_BASE, dataVal); // transmit data
           while(!(EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, UCTXIFG))); // wait for transmit buffer to empty itself
         }

        count++;

        }
     }

}
// Configure 3 second timer
void setupExpTimer(void){

    TB3CCR0 = 12288; // 3 sec * 4096 hz = 12288
    TB3CTL = TBSSEL_ACLK | MC_STOP | ID_3 | TACLR; // frequency = 4096 Hz
}

void msp_init(void){

    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // unlock ports
    setupLXT(); // setup up clock source
    setupUART(); // setup UART module
    setupSPI(); // setup SPI module
    setupADC();
    setupDAC();
    setupWatchDog(); // setup WDT
    setupExpTimer();
    exp = EXP_2;
    TB3CTL = TBSSEL_ACLK | MC_UP | ID_3 | TACLR; // frequency = 4096 Hz

}

int main(void)
{

    msp_init();


    while(1){

     if(exp == TRANS){
         sendDataOut(0,8); // send 8 data frames for Experiment 2
         sendDataOut(1,3); // send 3 data frames for Experiment 3
       }



    }

}

// Timer B1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_B0_VECTOR
__interrupt void toggleWDT(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_B0_VECTOR))) Timer1_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    P4OUT ^= BIT4; // toggle LED every 150 ms
}

// ADC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC_VECTOR))) ADC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
    {
        case ADCIV_NONE:
            break;
        case ADCIV_ADCOVIFG:
            break;
        case ADCIV_ADCTOVIFG:
            break;
        case ADCIV_ADCHIIFG:
            break;
        case ADCIV_ADCLOIFG:
            break;
        case ADCIV_ADCINIFG:
            break;
        case ADCIV_ADCIFG:
            if (adcseqnum == 11){
                mpptValues[i+16] = ADCMEM0;
            }

            else if(adcseqnum == 10){
                detValues[i+16] = ADCMEM0;
            }

            else if(adcseqnum == 9){
                detValues[i] = ADCMEM0;
            }

            else if(adcseqnum == 8){
                mpptValues[i] = ADCMEM0;
            }

            else if(adcseqnum == 3){
                lightTempValues[i] = ADCMEM0;
            }
            adcseqnum--;
            flag = 0;
            break;
        default:
            break;
    }
}

#pragma vector = TIMER3_B0_VECTOR
__interrupt void exp_ISR(){ // fires every seconds

    if(exp == EXP_2){ // run experiment
       run_exp2();
       if(measurement >= 45){
           exp = EXP_3;
       }
    }


    else{ // else run experiment 3
        run_exp3(exp3_tracker);
        exp3_tracker++;
        adcseqnum = 11;
        DAC_data += 254;
        SAC3DAT = DAC_data;
        if(measurement >= 45){
            exp = TRANS;
            TB3CTL = TASSEL__ACLK | MC__STOP | TACLR; // stop timer
        }

    }

}

