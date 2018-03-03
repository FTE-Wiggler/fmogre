

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


#define LFOSWITCH PORTAbits.RA8
#define RESOLUTIONSWITCH PORTBbits.RB4
#define SAMPLESWITCH PORTAbits.RA4

//     FM state variables
volatile unsigned long curbasephase;
volatile long curpitchval;
volatile long curpitchincr;
volatile long curfreqmod;
volatile long cvpm_dv, old_cvpm, older_cvpm, cvpm_predicted, cvpm_errpredmult;
volatile long curphasemod;
volatile long curaltphasemod;
volatile long curresolution;
volatile long curfbgain;
//     AD converter stuff
int which_adc;
unsigned long sinevalue;
volatile unsigned long final_phase_fm_fb_pm;
volatile unsigned long final_phase_fmpm;
volatile unsigned long final_phase_fm_feedback;
volatile long dac1la, dac1lb;
volatile long phase_shift;
volatile long phaselowpass;
volatile char oldhardsync;

static unsigned int dma_eng_addr;

volatile unsigned int cvdata[16] __attribute__ ((space(dma),aligned(256)));
volatile unsigned adc_data;
volatile unsigned adc_chan;


#define cvpitch (cvdata[7]) //was 0 TIMO EDIT
#define cvfb (cvdata[1])
#define cvfm (cvdata[2])
#define cvpm (cvdata[3])
#define cvpmknob (cvdata[4])
#define cvfmknob (cvdata[5])
#define cvfbknob (cvdata[6])
#define cvpitchknob (cvdata[8])   //was 0 and before that 7 TIMO EDIT

#define ADC_FIRST (0)
#define ADC_LAST (7)
#define PBUF_LEN 4096


//     Test point defines
#define TESTPOINT1 PORTCbits.RC6
#define TESTPOINT2 PORTCbits.RC7
#define TESTPOINT3 PORTCbits.RC8

//   Unless we use __attribute__ ((far)) we would need to use large memory model for this to work.
__attribute__ ((far)) unsigned int pbuf [PBUF_LEN];   
volatile unsigned pbindex = 0;

#define SYNC_IN (PORTAbits.RA9)

#include "wavetable.h"


void __attribute__((__interrupt__,__auto_psv__)) _DAC1LInterrupt(void)
{
}

void __attribute__((__interrupt__,__auto_psv__)) _DAC1RInterrupt(void)
{
    IFS4bits.DAC1RIF = 0;             //    clear the interrupt
    TESTPOINT2 = 1;                   //   show we're in DAC1RInterrupt

    {
        pbuf[pbindex] = (((unsigned long) 4095) - cvfb) ;
        if (PORTAbits.RA9 == 0) {pbindex++; };   // is SYNC / FREEZE on or off? ///TIMO edit  RB9->RA9
        if (pbindex >= PBUF_LEN) pbindex = 0;
    }
    
    curbasephase = curbasephase + curpitchincr ;
    curbasephase = curbasephase + curfreqmod;

    sinevalue = SAMPLESWITCH ? 
         pbuf[0x00000FFF & (curbasephase >> 20)]
        : sine_table [0x00000FFF & (curbasephase >> 20)] ;


    final_phase_fm_feedback = 0x00000FFF &
                    (
                        (
                            (curbasephase >> 20)    //  native base phase
                                +                       // plus operator feedback - FB jack is also sample in
                            //(((sinevalue - 32767 ) * curfbgain )>> 12 ) 
                            (((sinevalue - 32767 ) *
                                ((((long) (SAMPLESWITCH ? 4096 
                                                   : (4095 - cvfb)) * cvfbknob)>> 12 ))
                                ) >> 12 )//  (((long)cvfb * cvfbknob)) >> 12))
                         )
                    );

    if (cvpm != old_cvpm)
    {
        older_cvpm = old_cvpm;
        cvpm_predicted = older_cvpm;
        old_cvpm = cvpm;
    }    
    else
    { 
        cvpm_predicted = (cvpm_predicted + old_cvpm) >> 1;   // use this for 26.4 KHz max freq 
    }
    
    curphasemod = (( (long)2047 - cvpm_predicted) * cvpmknob) >> 10;
    curresolution = ((cvpmknob) * ((long)4095 - cvpm)) >> 12;  // works jack goes +-5 

    final_phase_fm_fb_pm = 0x00000FFF &
                    (
                         // native base phase
                             final_phase_fm_feedback
                            +
                             (RESOLUTIONSWITCH ?  0 : curphasemod)
                    );

    DAC1RDAT = sine_table [0x00000FFF & final_phase_fm_feedback];

    unsigned int fpffpp, dist1, dist2;
    fpffpp = 0xfff & (final_phase_fm_fb_pm + 2048);
    dist1 = (abs (final_phase_fm_fb_pm - pbindex));
    dist1 = (dist1 ) > 256 ?  16 : (dist1 >> 4);
    dist2 = 16 - dist1;
    dac1la = SAMPLESWITCH ?  
            pbuf [final_phase_fm_fb_pm] * dist1  // was enabled
            + pbuf [fpffpp] * dist2  //  was enabled 
            : sine_table [0X00000fff & (final_phase_fm_fb_pm)];
            
    {
        //   DANGER DANGER DANGER!!!  Use muldiv decimation only with DAC dividers 
        //   of 6 or greater!   Otherwise, the interrupt cannot keep up with the
        //   demand of the DAC oversampling hardware and you'll underrun which makes
        //   lots of negative-going pulses.  Not A Good Thing!

        //   Next step:  decimation / resolution reduction - uses integer divide
        //   then multiply by same amount to reduce the resolution.
        dac1lb = (((dac1la - 32768) / ((curaltphasemod)>> 8))
                * ((curaltphasemod)>>8))
                + 32768;
    }
    DAC1LDAT =  RESOLUTIONSWITCH ? dac1lb : dac1la;
   
    
    //   Hard sync out - tried doing this in base loop, too much jitter.  
    //    Even here, there's a lag of about 0.2 millisecond in the DAC path
    //    due to the 256x oversampling versus the direct path here for SYNC OUT
    //    Note that Hard Sync IN changes if we're in granular/sampling mode to
    //    become _FREEZE_INPUT_SAMPLES_
    if (SAMPLESWITCH)
    {  PORTBbits.RB8 = pbindex < 0x000000FF; }
    else
    {  PORTBbits.RB8 =  final_phase_fm_feedback > 0x00000800; }
    
    
    TESTPOINT2 = 0;
    return;

}


int main(int argc, char** argv) 
{

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
    ACLKCONbits.SELACLK = 0;    //   Aux oscillator from Main Fosc;
    ACLKCONbits.APSTSCLR = 6;   //  was 6: Divide by 2 - gets 20 MHz to the DAC;
                                //  use 6 to get 83333 hz DAC output rate (measured @ 40 MHz inst)
    ACLKCONbits.ASRCSEL = 0;    //    use primary clock as source (but doesn't matter)

    long int quantum;
    
                     //  0x1 = ~ 1/15 Hz (1 cycle per 15 seconds)
    quantum = 0x2400;   //  0x24000 = ~ 10 KHz; 0x35000 =~ 15 KHz


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

    //   Set up the DACs
    DAC1CONbits.DACEN = 1;    // enable the audio dac
    DAC1CONbits.AMPON = 1;    // enable the output amplifier
    DAC1CONbits.FORM = 0;     //  unsigned data (0 = unsigned data)
    //
    //    CHANGE THE DACFDIV TO CHANGE OUTPUT SAMPLING SPEED!!!
    //     The following pitch-DACFDIV values are relative, of course.
    //    (Note, this is not sample RATE.  This is sine wave rendered)
    //    8 = 14.7 KHz max pitch
    //    7 = 16.5 KHz max pitch
    //    6 = 18.9 KHz max pitch, 50 KHz phase update, fastest for MULDIV 
    //    5 = 22.0 KHz max pitch, 50 KHz phase update
    //    4 = 26.4 KHz max pitch, 50 KHz phase update
    //    3 = 33.0 KHz max pitch, 33 KHz phase update
    //    2 = 44.07 kHz max pitch, 28 KHz phase update 
    //    1 .... antialias malfunction.  Don't use this.
    //    To be honest, it sounds really good at either 3 or 4 to me if you want
    //    dogs to hear it, and just fine at 6, which allows MULDIV-decimation.
    
    DAC1CONbits.DACFDIV = 6;  // (was 3, then 6, then 8) divide Fosc to drive 
                              //  interpolator. 3 = 83.333KHz @40MIPS
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
    PR3 = 100;                 // 100 = prescale to 400 KHz
    //PR3 = 80;                  //  80 = prescale to 500 KHz  (maximum rated)
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

    //   Zero the buffer index for starters.
    pbindex = 0;
    cvpm_errpredmult = 0;

    IEC4bits.DAC1RIE = 1;      //  enable the right channel DAC FIFO interrupt
    IPC19bits.DAC1RIP = 5;     //  set the FIFO interrupt priority (7 is max)

    while (1)   // Loop Endlessly - Execution is interrupt driven
    {

        //   Turns out that if you just toggle a bit fast the LED doesn't
        //   respond AT ALL (RC issues - the 220 ohm ballast resistance and the LED's
        //   capacitance form an RC filter and the output voltage never gets high
        //   enough to turn on the LED).  So we use TRIS (tristate) instead.
        //      Use this for normal operation (on in first 1/2 of wave)
        TRISBbits.TRISB5 = 0x7FF < (0x00000FFF & (curbasephase >> 20 ));


        curpitchval = cvpitchknob + ((long)2047 - cvpitch) ;
        
        //   use exp_table to get exp freq resp, or if RA8 (aka FLOSWITCH) is turned off
        //   then we're in LFO mode and we use the value rightshifted 12 bits (very low
        //   frequency).
        curpitchincr = LFOSWITCH ?
            ((exp_table [
              (curpitchval < 0) ? 0 :
                (curpitchval > 0x00000FFF) ? 0x00000FFF : curpitchval]) >> 12)
            :
            (exp_table [
              (curpitchval < 0) ? 0 :
                (curpitchval > 0x00000FFF) ? 0x00000FFF : curpitchval]);


        curfreqmod = cvfmknob * (((long)2047) - cvfm) << 6;

        //    Output RB6 high if we have negative frequency.  Note because
        //    of capacitance issues, we ALWAYS have RB6 on, but rather
        //    just turn on and off the tristate (TRISbits) for RB6
        PORTBbits.RB6 = 1;
        TRISBbits.TRISB6 = curpitchincr + curfreqmod < 0 ? 0 : 1;

        curphasemod = ((((long)2047 - cvpm) * cvpmknob) >> 10);

        curaltphasemod = (( (long)2047 - cvpm) * cvpmknob) + 256;

        //   Output RB7 driven high if we have negative phase.   Note that
        //   because of capacitance issues, we can't just output the bit; we
        //   have to change the tristate (TRISbits) to hi-Z the output.
        PORTBbits.RB7 = 1;
        TRISBbits.TRISB7 = curpitchincr + (curphasemod << 13) < 0 ? 0 : 1 ;


        //   Do we have a HARD SYNC IN request on pin RB9?
        //   GROT GROT GROT note that this hacky hysteresis could / should
        //   really be done in an interrupt.  Maybe later....
        if (PORTAbits.RA9 == 1 && SAMPLESWITCH == 0 )  //TIMO EDIT RB9->RA9
        {
            if (oldhardsync == 0)
            {
                curbasephase = 0;
                oldhardsync = 1;
            }
        }
        else
            oldhardsync = 0;


        curfbgain = ((4095 - cvfb) * cvfbknob) >> 12;
    }
}

