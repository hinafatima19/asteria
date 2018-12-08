#include <msp430.h> 


/**
 * Main payload software rountine for Project Asteria (WIP) 
 */

enum packet_seq {HEADER,DATA}
enum exp_status {INIT, IDLE, EXP_2, EXP_3}
int mpptValues[35]; // MPPT Packet
int detValues[35]; // DET Packet
int uhfValues[35]; // UHF Packet

enum exp_status exp;
enum packet_seq packet = HEADER;
/*
 *
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


void msp_init(void){

    exp = INIT;
    P2SEL0 |= BIT0 | BIT1; // USCI_A0_UART component select
    P2SEL1 &= ~(BIT0|BIT1);

    PJSEL0 |= BIT4 | BIT5; // XT1

    /**
     * Add ADC CONFIGURATION
     */

    /*
     * Add TIMER FOR EXPERIMENT #2 CONFIGURATION
     */



    PM5CTL0 &= ~LOCKLPM5; // unlock power manager

    // XT1 Setup
    CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
    CSCTL2 = SELA__VL0CLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers
    CSCTL4 &= ~LFXTOFF;

    do
    {
      CSCTL5 &= ~LFXTOFFG;                    // Clear XT1 fault flag
      SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
    CSCTL0_H = 0;                             // Lock CS registers

    // UART component configuration

    UCA0CTLW0 = UCSWRST;                      // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK

    // Baud rate and other parameters taken from table of reccomended settings
    // Here our BRCLK = SMCLK so
    // frBRCLK = 1 MHz
    // Baudrate = 9600
    // N = 1000000 / 9600 = 6
    // UCBRx = 6
    // UCBRFx = 8
    // UCBRSx = 0x92

    UCA0BR0 = 6;
    UCA0BR1 = 1;
    UCA0MCTLW = UCOS16 | UCBRF_8 | 0x9200;

    //UCA0IE |= UCTXIE | UCRXIE; // enable transmit and receive interrupts not sure if we want to use blocking loops or interrupts for UART
                                 // based on the rest of our software, its probably best to use blocking loops.

}

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	msp_init();
	exp = IDLE;

	while(1){

	 switch(exp){

	 case IDLE: // keep sleeping in low power mode
	     __bis_SR_register(LPM3_bits);
	     break;

	 case EXP_2:
	     sendExperiment2Data();
	     exp = IDLE;
	     break;

	 case EXP_3:
	     sendExperiment2Data();
	     exp = IDLE;
	     break;

	 }

	}

	return 0;



}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
      while(!(UCA0IFG&UCTXIFG));
      UCA0TXBUF = UCA0RXBUF;
      __no_operation();
      break;
    case USCI_UART_UCTXIFG: break;
        // SEND DATA
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
  }
}
