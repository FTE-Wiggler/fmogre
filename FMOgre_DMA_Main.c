

#include <stdio.h>
#include <p33FJ128GP804.h>
#include "dsp.h"
#include <stdint.h>
#include <p33Fxxxx.h>

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

_FOSCSEL (FNOSC_PRIPLL & IESO_ON )
_FOSC (POSCMD_XT & OSCIOFNC_OFF & IOL1WAY_OFF & FCKSM_CSECME)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR16            // POR Timer Value (16ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)


static unsigned int dma_eng_addr;

// ADC data is in [0..4095]
volatile int cvdata[16] __attribute__ ((space(dma),aligned(256)));

#define CVDATA_FB        1
#define CVDATA_FM        2
#define CVDATA_PM        3
#define CVDATA_PMKNOB    4
#define CVDATA_FMKNOB    5
#define CVDATA_FBKNOB    6
#define CVDATA_PITCH     7
#define CVDATA_PITCHKNOB 8

// Pins
#define LFOSWITCH           PORTAbits.RA8
#define RESOLUTIONSWITCH    PORTBbits.RB4
#define SAMPLESWITCH        PORTAbits.RA4
#define SYNC_IN             PORTAbits.RA9
#define TESTPOINT1          PORTCbits.RC6
#define TESTPOINT2          PORTCbits.RC7
#define TESTPOINT3          PORTCbits.RC8

#define PBUF_LEN 4096

//   Unless we use __attribute__ ((far)) we would need to use large memory model for this to work.
__attribute__ ((far)) unsigned int pbuf [PBUF_LEN];   


#include "wavetable.h"

void __attribute__((__interrupt__,__auto_psv__)) _DAC1LInterrupt(void)
{
}

// Variables that cross interrupt/non-interrupt contexts.
long curbasephase;     			// Current oscillator phase.
long phaseincr_pitch;			// Includes only v/oct component.
long phaseincr;                 // Includes v/oct and fm components.
long phasemod;					// Phase modulation.
long phasemod_last;				// Previous phase modulation.

void __attribute__((__interrupt__,__auto_psv__)) _DAC1RInterrupt(void)
{   
    static unsigned pbindex = 0;

    IFS4bits.DAC1RIF = 0;             //    clear the interrupt
    TESTPOINT2 = 1;                   //   show we're in DAC1RInterrupt

    long cvfb = 4095 - cvdata[CVDATA_FB];           // Still positive value.
    long cvfbknob = cvdata[CVDATA_FBKNOB];          // Positive value.
    
    {
        pbuf[pbindex] = cvfb;
        if (PORTAbits.RA9 == 0) {pbindex++; };   // is SYNC / FREEZE on or off? ///TIMO edit  RB9->RA9
        if (pbindex >= PBUF_LEN) pbindex = 0;
    }
    
    curbasephase += phaseincr;

    long sinevalue = SAMPLESWITCH ? 
         pbuf[0x00000FFF & (curbasephase >> 20)]
        : sine_table [0x00000FFF & (curbasephase >> 20)] ;

    sinevalue -= 32767; // Shift to be symmetric about zero.

    long fb_phaseshift;
    
    if (SAMPLESWITCH)
        fb_phaseshift = sinevalue * 4096;
    else
        fb_phaseshift = ((sinevalue * cvfb) >> 4) * cvfbknob;  // TODO: better precision if we can get 64 bit intermediate result.


    long final_phase_fm_feedback = curbasephase + fb_phaseshift;

    // =================  FM Output =======================================
    DAC1RDAT = sine_table [0x00000FFF & (final_phase_fm_feedback >> 20)];
    // ===================================================================

	phasemod_last = phasemod;

    int pm_jack = 2047 - cvdata[CVDATA_PM];   // TODO: low pass filter this.
    int pm_knob = cvdata[CVDATA_PMKNOB];      // Positive value.  TODO: filter the shit out of this.

	if (RESOLUTIONSWITCH) {
		phasemod = 0;
	} else {
		phasemod = ((long)pm_jack * pm_knob) << 9; // Does not need to scale with pitch frequency.
	}
	
    long final_phase_fm_fb_pm = final_phase_fm_feedback + phasemod;
    int foobar = 0x0FFF & (int)(final_phase_fm_fb_pm >> 20);

    long dac1la;
    if (SAMPLESWITCH) {
        unsigned int fpffpp, dist1, dist2;
        fpffpp = 0xfff & (foobar + 2048);
        dist1 = (abs (foobar - pbindex));
        dist1 = (dist1 ) > 256 ?  16 : (dist1 >> 4);
        dist2 = 16 - dist1;
        dac1la = pbuf[foobar] * dist1  +  pbuf[fpffpp] * dist2;
    } else {
        dac1la = sine_table[foobar];
        
        // TEST CODE -- this shows there is noise in the ADC conversion.
        //dac1la = ((cvpm+2047) * cvpmknob) >> 8;
    }
  
    if (RESOLUTIONSWITCH)
    {
		long altphasemod = (long)pm_jack * pm_knob + 256;
		
        //   DANGER DANGER DANGER!!!  Use muldiv decimation only with DAC dividers 
        //   of 6 or greater!   Otherwise, the interrupt cannot keep up with the
        //   demand of the DAC oversampling hardware and you'll underrun which makes
        //   lots of negative-going pulses.  Not A Good Thing!

        //   Next step:  decimation / resolution reduction - uses integer divide
        //   then multiply by same amount to reduce the resolution.
        long dac1lb = (((dac1la - 32768) / ((altphasemod)>> 8))
                * ((altphasemod)>>8))
                + 32768;
        DAC1LDAT = dac1lb;  // FM + PM Output
    } else {
        DAC1LDAT = dac1la;  // FM + PM Output
    }
    
    //   Hard sync out - tried doing this in base loop, too much jitter.  
    //    Even here, there's a lag of about 0.2 millisecond in the DAC path
    //    due to the 256x oversampling versus the direct path here for SYNC OUT
    //    Note that Hard Sync IN changes if we're in granular/sampling mode to
    //    become _FREEZE_INPUT_SAMPLES_
    if (SAMPLESWITCH)  
        PORTBbits.RB8 = pbindex < 0x000000FF;
    else
        PORTBbits.RB8 =  final_phase_fm_feedback > 0x80000000UL;
    
    
    TESTPOINT2 = 0;
    return;
}



int main(int argc, char** argv) 
{

    static int oldhardsync;

    // Configure Oscillator to operate the device at 40MIPS
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 7.37M*43/(2*2)=79.22Mhz for ~40MIPS input clock

    //   Current crystal is 4 MHz so the below actually clocks at 41 MIPS
    //  Yeah overclocking!!!   Set PLLFBD to 79 for an accurate 40 MIPS.
    //   which is 80 MHz on the PLL clock output
      CLKDIVbits.PLLPRE=0;		// N2=2
      PLLFBD = 80 ;			// Set this to 79 for an accurate 80 mips
      CLKDIVbits.PLLPOST=0;		// N1=2
                                        //  4MHz xtal + 0/80/0 yields ~40 MHz inst speed (measured)

    // Disable Watch Dog Timer
      RCONbits.SWDTEN=0;

    // Wait for PLL to lock
    while(OSCCONbits.LOCK!=1) {};

    //   Set up aux oscillator channel
    ACLKCONbits.SELACLK = 0;    //   Aux oscillator from Main Fosc (80 MHz as set above);
    ACLKCONbits.APSTSCLR = 6;   //   6: Divide by 2 - gets 40 MHz to the DAC;
    ACLKCONbits.ASRCSEL = 0;    //    use primary clock as source (not relevant when SELACLK = 0)


    //   Set up which pins are digital in and out
    TRISA = 0x0;    //   Inputs on RA0-1 (ADC 0 and 1)
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA4 = 1;   //  RA4 is switch 3(sine versus sample)
    TRISAbits.TRISA8 = 1;   //  RA8 is switch 1 (VCO versus LFO)
    TRISAbits.TRISA9 = 1;  //   Pin RA9 is SYNC IN. TIMO EDIT


    TRISC = 0x0;    //   Inputs on RC0-1 (ADC 6 and 7)
    TRISCbits.TRISC0 = 1;
    TRISCbits.TRISC1 = 1;
    TRISCbits.TRISC2 = 1;//TIMO EDIT (to use an8 in stead of an0)
    TRISB = 0x0;    //   Inputs on RB0-3 (ADC 2,3,4,5), RB8-9 (switches)RB12-15 (DACs)
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB4 = 1;  //  RB4 is switch 2
//   Pins RB5, RB6, and RB7 are the LED ports 
    TRISBbits.TRISB5 = 0;  //  RB5 is heartbeat (final out positive); we toggle TRIS.
    PORTBbits.RB5 = 1;
    TRISBbits.TRISB6 = 0;  //  RB6 is negative frequency
    PORTBbits.RB6 = 1;
    TRISBbits.TRISB7 = 0;  //  RB7 is negative phase
    PORTBbits.RB7 = 1;
    TRISBbits.TRISB8 = 0;  //   Pin RB8 is SYNC OUT,
    //TRISBbits.TRISB9 = 1;  //   Pin RB9 is not SYNC IN. TIMO EDIT
    TRISBbits.TRISB12 = 1;
    TRISBbits.TRISB13 = 1;
    TRISBbits.TRISB14 = 1;
    TRISBbits.TRISB15 = 1;

    //   Use port RC6 to see stuff like ADC and DAC interrupts, so output...
    TRISCbits.TRISC6 = 0;     //  Test Point 1
    TRISCbits.TRISC7 = 0;     //  Test Point 2
    TRISCbits.TRISC8 = 0;     //  Test Point 3   


    // Power-off all unused peripherals.
    PMD1bits.T5MD = 1;      // Timers
    PMD1bits.T4MD = 1;
    PMD1bits.T3MD = 0;      // Used to clock ADC.
    PMD1bits.T2MD = 1;
    PMD1bits.T1MD = 1;
    PMD1bits.DCIMD = 1;     // DCI
    PMD1bits.I2C1MD = 1;    // I2C
    PMD1bits.U2MD = 1;      // Uarts
    PMD1bits.U1MD = 1;
    PMD1bits.SPI2MD = 1;    // SPIs
    PMD1bits.SPI1MD = 1;
    PMD1bits.C1MD = 1;      // ECAN
    PMD1bits.AD1MD = 0;     // ADC is used.
    
    PMD2bits.IC8MD = 1;     // Input Captures
    PMD2bits.IC7MD = 1;
    PMD2bits.IC2MD = 1;
    PMD2bits.IC1MD = 1;
    PMD2bits.OC4MD = 1;     // Output Compares
    PMD2bits.OC3MD = 1;
    PMD2bits.OC2MD = 1;
    PMD2bits.OC1MD = 1;

    PMD3bits.CMPMD = 1;     // Comparator
    PMD3bits.RTCCMD = 1;    // Real time clock
    PMD3bits.PMPMD = 1;     // Parallel master port
    PMD3bits.CRCMD = 1;     // CRC generator
    PMD3bits.DAC1MD = 0;    // DAC is used. (duh)


    //   Set up the DACs
    DAC1CONbits.DACEN = 1;    // enable the audio dac
    DAC1CONbits.DACSIDL = 0;  // continue operation in idle mode
    DAC1CONbits.AMPON = 1;    // enable the output amplifier
    DAC1CONbits.FORM = 0;     //  unsigned data (0 = unsigned data)
    
    //
    //    CHANGE THE DACFDIV TO CHANGE SAMPLING RATE
    //        Fs = Fcy / (256 * (DACFDIV + 1))
    //
    //    DACFDIV      Fs (assuming Fcy = 40 MHz)
    //    ---------------------------------------
    //      1          78125
    //      2          52083.3333
    //      3          39062.5000
    //      4          31250
    //      5          26041.6666
    //      6          22321.42857
    
    #define MY_DACFDIV  2
    
    DAC1CONbits.DACFDIV = MY_DACFDIV; 
    DAC1STAT = 0xFFFF;        //  everything on
    DAC1STATbits.LITYPE = 1;  // 0 means interrupt on Left LIFO not full
    DAC1STATbits.RITYPE = 1;  // 0 means interrupt on Right LIFO not full
    DAC1STATbits.LMVOEN = 0;  //  left channel midpoint output off
    DAC1STATbits.RMVOEN = 0;  //  right channel midpoint output off


        //   Do not use the ADC interrupt.   Instead, TIMER3 starts it, and 
    //   the DMA engine moves the output into the cvdata array.
    //    IEC0bits.AD1IE = 1;        //  enable the ADC interrupt.
    //    IPC3bits.AD1IP = 2;        //  set priority for the ADC interrupt (7 is max)

    //   Set up DMA channel 0 for the ADC 
    DMA0CONbits.CHEN = 0;           //  Disable DMA channel 0 during setup
    DMA0CONbits.SIZE = 0;           //  0 = word transfers
    DMA0CONbits.DIR = 0;            //  0 = read from peripheral, write to RAM
    DMA0CONbits.HALF = 0;           //  0 = wait for full transfer before interrupt
    DMA0CONbits.NULLW = 0;          //  0 = normal write 
    DMA0CONbits.AMODE = 2;          //  2 = peripheral indirect addressing
    DMA0CONbits.MODE = 0;           //  0 = continuous transfer, no ping-pong
    DMA0PAD = (int) &ADC1BUF0;     //  source the data from the ADC
    DMA0CNT = 0;                //   How many transfers (DMA0CNT=0 --> 1 transfer)
    DMA0REQbits.IRQSEL = 13;        //  ADC1DAT0 is #13 (see manual table 8.1)
    dma_eng_addr = __builtin_dmaoffset(&cvdata);  // offset for DMA engine
    DMA0STA = dma_eng_addr;     //  DMA engine's buffer address - not CPU's
    DMA0STB = 0;                //   No DMA secondary register
    IFS0bits.DMA0IF = 0;        //   Clear DMA0's interrupt flag
    IEC0bits.DMA0IE = 0;        //   Clear the DMA interrupt enable
    DMA0CONbits.CHEN = 1;        //   Enable the DMA channel.   Off you go!
    
    //    Set up the ADCs
    //    _ADON = 1;                //  turn on the ADC
    AD1CON1bits.ADON = 0;       //   AD converter off while configuring
    AD1CON1bits.AD12B = 1;     //  1 = 12-bit mode
    AD1CON1bits.FORM = 0;      //  output as unsigned 12 bit integer
    AD1CON1bits.SSRC = 2;      //  TIMER3 ends sampling and starts conversion
    AD1CON1bits.ASAM = 1;      //  Sample again right after conversion
    AD1CON1bits.ADSIDL = 0;    //  keep it on even during idle
    AD1CON1bits.ADDMABM = 0;   //  Module provides real addr (1) or s/g (0) to DMA
    AD1CON1bits.DONE = 0;      //  set ADC to not be done yet (HW sets to 1 on completion)

    AD1CON2bits.CSCNA = 1;     //  1=scan inputs, 0=do not scan inputs
    AD1CON2bits.VCFG = 1 ;
    AD1CON2bits.CHPS = 0;      //  use just 1 ADC channel
    AD1CON2bits.SMPI = 7;      //  (was 7)increment DMA adddress after N+1 converts
    AD1CON2bits.BUFM = 0;      //  always start buffer fill at 0
    AD1CON2bits.ALTS = 0;      //  always use sample A

    //   ADC clock must nominally be at least 117 nS long per cycle
    AD1CON3bits.ADRC = 0;      //  0 = sysclock; 1=use the ADC's internal RC clock
    AD1CON3bits.ADCS = 8;      // (was 7, then 4, then 2)  divide 80 mHz by this plus 1
    //AD1CON3bits.SAMC = 1;      //  Auto sample time bits (ignored here)

    AD1CON4bits.DMABL = 0;     //  DMA buffer length 1 word per analog input
    
    AD1CHS0bits.CH0NB = 0;     //  channel B negative input is Vref-
    AD1CHS0bits.CH0SB = 0;     //  channel B pos input is input AD0
    AD1CHS0bits.CH0NA = 0;     //  channel A negative input is Vref-
    AD1CHS0bits.CH0SA = 0;     //  channel A positive input is input AD0
    AD1CSSL = 0x01FE;            //  AN0-7 enabled for scan  //TIMO EDIT was 00FF (to use an8 in stead of an0)
    AD1PCFGL = 0xFE00;         //  enable ADC on low 8 AD pins. //TIMO EDIT was FF00 (to use an8 in stead of an0)
    IFS0bits.AD1IF = 0;        //  Clear the AD interrupt flag
    IEC0bits.AD1IE = 0;        //  Do not enable AD interrupt 
    AD1CON1bits.ADON = 1;      // turn on the ADC

    //   Set up the TIMER3 to keep kicking the ADC.   We use TIMER3 because
    //   only timer3 has a direct wire to kick the ADC into sample/convert.
    TMR3 = 0x0000;
    //              PR3 is the scale factor for the ADC converter.  400 KHz
    //  sounds pretty good (that's update at 50 KHz per input), PR3=100.
    //   Yes, there is crosstalk from one channel to another.  But it's 
    //   not improved by slowing down the ADC (yes, I've tested this all the way
    //   down to 8 KHz sample rate and going slower doesn't help; even more
    //   amazingly it doesn't sound that bad at all even though the scope trace
    //   is utter crud.)
    //PR3 = 4999;                // 4999 = prescale to 125 uSec (8 KHz)
    //PR3 = 250;                 // 250 = prescale to 160 KHz
    //PR3 = 200;                  //  200 = prescale to 200 KHz
    //PR3 = 150;
    //PR3 = 100;                 // 100 = prescale to 400 KHz
    //PR3 = 80;                  //  80 = prescale to 500 KHz  (maximum rated)
    
    // TO SAMPLE SYNCHRONOUSLY WITH THE DAC:
    //        * assuming we have set the DAC oscillator to be Fcy (40 MHz)
    //        * then the DAC division factor is given by (DACFDIV+1) * 256
    //        * since we are sampling 8 inputs, we want to use (DACFDIV+1) * 256/8
    //        * set the period register to 1 less than the desired divider
    //        * eg: PR3 = (DACFDIV+1) * 32 - 1 
    PR3 = (MY_DACFDIV+1) * 32 - 1;
    
    T3CONbits.TSIDL = 0;       // keep timing in idle
    IFS0bits.T3IF = 0;         // Clear T3 interrupt
    IEC0bits.T3IE = 0;         // Disable T3 interrupt
    T3CONbits.TON = 1;             // start the timer
    
            
    
    
    //   Zeroize the ADC inputs in preparation for those that aren't updated
    {
        int i;
        for (i = 0; i < 12; i++)
            cvdata[i] = 0;
    }

    IEC4bits.DAC1RIE = 1;      //  enable the right channel DAC FIFO interrupt
    IPC19bits.DAC1RIP = 5;     //  set the FIFO interrupt priority (7 is max)

    while (1)   // Loop Endlessly - Execution is interrupt driven
    {
        Idle(); // Go to low power mode until woken up by the DAC interrupt.
        
        //   Turns out that if you just toggle a bit fast the LED doesn't
        //   respond AT ALL (RC issues - the 220 ohm ballast resistance and the LED's
        //   capacitance form an RC filter and the output voltage never gets high
        //   enough to turn on the LED).  So we use TRIS (tristate) instead.
        //      Use this for normal operation (on in first 1/2 of wave)
        TRISBbits.TRISB5 = 0x7FF < (0x00000FFF & (curbasephase >> 20 ));


        int curpitchval = cvdata[CVDATA_PITCHKNOB] + (2047 - cvdata[CVDATA_PITCH]);
		if (curpitchval < 0)
			curpitchval = 0;
		else if (curpitchval > 4095)
			curpitchval = 4095;
        
        //   use exp_table to get exp freq resp, or if RA8 (aka FLOSWITCH) is turned off
        //   then we're in LFO mode and we use the value rightshifted 12 bits (very low
        //   frequency).
		long pitchincr = exp_table[curpitchval]; // 31 bit
		if (LFOSWITCH)
			pitchincr = pitchincr >> 12;
			

		int fm_jack = 2047 - cvdata[CVDATA_FM];
		long freqmod = (long)fm_jack * (int)(cvdata[CVDATA_FMKNOB]);  // max 8387000 or 24 bits
		freqmod = (freqmod >> 8) * (pitchincr >> 13);  	// Scales with pitch frequency.
        
        __builtin_disi(0x3FFF); // Non-atomically setting variables used by interrupt handler.
		phaseincr_pitch = pitchincr;
        phaseincr = pitchincr + freqmod;
        __builtin_disi(0x0000);

        //    Output RB6 high if we have negative frequency.  Note because
        //    of capacitance issues, we ALWAYS have RB6 on, but rather
        //    just turn on and off the tristate (TRISbits) for RB6
        PORTBbits.RB6 = 1;
        TRISBbits.TRISB6 = phaseincr < 0 ? 0 : 1;

        //   Output RB7 driven high if we have negative frequency due to phase modulation. Note that
        //   because of capacitance issues, we can't just output the bit; we
        //   have to change the tristate (TRISbits) to hi-Z the output.
        PORTBbits.RB7 = 1;
        TRISBbits.TRISB7 = (pitchincr + (phasemod - phasemod_last) < 0) ? 0 : 1 ;


        //   Do we have a HARD SYNC IN request on pin RB9?
        //   GROT GROT GROT note that this hacky hysteresis could / should
        //   really be done in an interrupt.  Maybe later....
        if (PORTAbits.RA9 == 1 && SAMPLESWITCH == 0 )  //TIMO EDIT RB9->RA9
        {
            if (oldhardsync == 0)
            {
                __builtin_disi(0x3FFF); // Non-atomically setting variable used by interrupt handler.
                curbasephase = 0;
                __builtin_disi(0x0000);
                oldhardsync = 1;
            }
        }
        else
            oldhardsync = 0;

    }
}

