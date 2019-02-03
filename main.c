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

enum exp_status exp;
enum packet_seq packet = HEADER;

/*
 *  To be completed:
 *  -- SPI read from UHF chip onboard Payload
 *  -- insert each byte into array containing experiment 2 data
 */
void UHFRead() { //

}

/*
 * To be completed:
 *  -- read DET voltage or current value from the ADC and place it into the array containing DET values
 */
void addDETValue(){

}


/*
 * To be completed:
 *  -- read MPPT voltage or current value from the ADC and place it into the array containing MPPT values
 */
void addMPPTValue(){

}

/**
 * To be completed
 * -- send experiment 2 data to OBC using UART module
 */
void sendExperiment2Data(void){

}


/**
 * To be completed
 * -- send experiment 3 data to OBC using UART module
 */
void sendExperiment3Data(int mppt_data[], int det_data[]){

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

void setupUART(){ // initialize UART module


    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_UCA0TXD, // configure TX pin
                                               GPIO_PIN_UCA0TXD,
                                               GPIO_FUNCTION_UCA0TXD);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_UCA0RXD, // configure RX pin
                                               GPIO_PIN_UCA0RXD,
                                               GPIO_FUNCTION_UCA0RXD);

    // Baud rate and other parameters taken from table of recommended settings
    // Here our BRCLK = SMCLK so
    // frBRCLK = 1 MHz
    // Baudrate = 9600
    // N = 1000000 / 9600 = 6
    // UCBRx = 6
    // UCBRFx = 8
    // UCBRSx = 0x20
    UCA0CTLW0 = UCSWRST;                      // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
    UCA0BRW = 6;
    UCA0MCTLW = UCOS16 | UCBRF_8 | 0x2000;
    UCA0CTLW0 &= ~UCSWRST;

}

void setupWatchDog(void){
    // initialize watchdog timer to interval of 16 secs
    WDT_A_initWatchdogTimer(WDT_A_BASE, WDT_A_CLOCKSOURCE_ACLK, WDT_A_CLOCKDIVIER_512K);
    WDT_A_start(WDT_A_BASE);
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

void setupADC(void){

}

void setupDAC(){

}

void sendDataOut(int experiment, int frames){

   int count = 0;

   if(experiment == 0){

      while(count < frames){
         int i;
         for(i = 0; i < 35; i++){
           while(!(P2IN & BIT3)){} // wait for busy flag
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



void msp_init(void){

    setupLXT(); // setup up clock source
    setupUART(); // setup UART module
    setupSPI(); // setup SPI module
    setupWatchDog(); // setup WDT

    exp = EXP_2;

    /**
     * Add ADC CONFIGURATION
     */

    /*
     * Add TIMER FOR EXPERIMENT #2 CONFIGURATION
     */

    PM5CTL0 &= ~LOCKLPM5; // unlock power manager
}

int main(void)
{
    //WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    msp_init();


    while(1){

     if(exp == TRANS){
         sendDataOut(0,8); // send 8 data frames for Experiment 2
         sendDataOut(1,3); // send 3 data frames for Experiment 3
       }

     WDT_A_resetTimer(WDT_A_BASE); // pet watchdog timer

    }

}
