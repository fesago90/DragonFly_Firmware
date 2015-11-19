#include "Common.h"
#include "FilterCoeffs.h"
#include "VMeas.h"

// CAREFUL, this data needs to be MANUALLY accessed by setting DSRPAG to 0x1 since
// DMARAM is located in upper 32KB of RAM
VoltageData VoltageDataBufferA __attribute__((space(dma), eds)) = {0xaaaa, 0xaaaa, 0xaaaa};
VoltageData VoltageDataBufferB __attribute__((space(dma), eds)) = {0xaaaa, 0xaaaa, 0xaaaa};

fractional  VMeasBattFilterDelay[LPF_5_1_25_50_SIZE]  __attribute__ ((space(ymemory), eds));
FIRStruct   VMeasBattLowPassFilter;
VoltageData VMeasCurrentFiltData;
uint16_t    VMeasFilteredSampleCount = 0;

void VMeas_Init_Pins();
void VMeas_Init_ADC();
void VMeas_Init_DMA();
void VMeas_Init_Timer();
void VMeas_Init_Filter();

void VMeas_Init()
{
    #if (VMEAS_EN == 1)
        VMeas_Init_Pins();
        VMeas_Init_ADC();
        VMeas_Init_DMA();
        VMeas_Init_Filter();
        VMeas_Start();

        uint16_t count = 0;
        while(count < 500) {
            count++;
            __delay_ms(1);
            VMeas_Filter_Current_Sample();
        }
        VoltageData vd = *(VMeas_Get_Current_Filt_Data());
        VMeas_Filter_Current_Sample();
        while(VMEAS_MIN_LIBATT > (unsigned int)vd.V_Batt) {
            vd = *(VMeas_Get_Current_Filt_Data());
            VMeas_Filter_Current_Sample();
        }
    #endif
}

/*
 * Set appropriate pins as input and enable analog functionality.
 */
void VMeas_Init_Pins()
{
    // Inputs
    _TRISbit(ADC_SERVO_PORT, ADC_SERVO_BIT) = 1;
    _TRISbit(ADC_3V_PORT, ADC_3V_BIT)       = 1;
    _TRISbit(ADC_BATT_PORT, ADC_BATT_BIT)   = 1;
    // Analog pins
    _ANSELbit(ADC_SERVO_PORT, ADC_SERVO_BIT) = 1;
    _ANSELbit(ADC_3V_PORT, ADC_3V_BIT)       = 1;
    _ANSELbit(ADC_BATT_PORT, ADC_BATT_BIT)   = 1;
}

/*
 * Configures for 12-bit, 1-channel scanning ADC operation with DMA. Sampling must be
 * manually started. Range is 0 to 3.0V.
 *
 */
void VMeas_Init_ADC()
{
    VMEAS_ADC_IE = 0;                     // Disable AD1 interrupt
    VMEAS_ADC_IF = 0;                     // Clear AD1 interrupt flag

    VMEAS_ADCON1.ADON    = 0;        // Turn module OFF

    VMEAS_ADCON4.ADDMAEN = 1;        // Conversion results are stored in buffer register for DMA transfers to RAM
    VMEAS_ADCON4.DMABL   = 0b001;    // Allocates 1 word of buffer to each analog input

    //AD1CON1bits.ADDMABM = 0;        // DMA buffers are written in order of conversion
    VMEAS_ADCON1.AD12B   = 1;        // 12-bit, 1 channel
    VMEAS_ADCON1.FORM    = 0b00;     // Unsigned integer data output format
    VMEAS_ADCON1.SSRCG   = 0;        // Sample clock source is not from PWM:
    VMEAS_ADCON1.SSRC    = 0b010;    // Timer3 compare ends sampling and starts conversion

    VMEAS_ADCON2.VCFG    = 0b000;    // Vrefh = Avdd (3.0 V), Vrefl = Avss (0.0 V)
    VMEAS_ADCON2.CSCNA   = 1;        // Enable channel scanning
    VMEAS_ADCON2.SMPI    = 2;        // DMA interrupt after 3rd sample

    VMEAS_ADCON3.ADRC    = 0;        // Clock derived from system clock
    VMEAS_ADCON3.ADCS    = 8;        // Tad = 9 * Tcy = 150 ns
    VMEAS_ADCON3.SAMC    = 7;       // Sample time = 7 * Tad

    //VMEAS_CSSL.CSS3    = 1;        // Selects AN3 for input scan
    //VMEAS_CSSL.CSS4    = 1;        // Selects AN4 for input scan
    //VMEAS_CSSL.CSS5    = 1;        // Selects AN5 for input scan

    VMEAS_CSSL.CSS0    = 1;        // Selects AN0 for input scan  --- VREF+?
    VMEAS_CSSL.CSS2    = 1;        // Selects AN0 for input scan  --- VBATT
    VMEAS_CSSL.CSS3    = 1;        // Selects AN3 for input scan  --- 3.3Servo

    VMEAS_ADCON1.ADON    = 1;        // Turn module ON
    __delay_us(30);                 // Provide ADC stabilization delay
}

/*
 * 
 */
void VMeas_Init_DMA()
{
    VMEAS_DMA_IE        = 0;        // Disable DMA2 interrupt
    VMEAS_DMA_IF        = 0;        // Clear DMA2 interrupt flag
    VMEAS_DMACON.CHEN   = 0;        // Disable DMA2 channel

    VMEAS_DMAPAD        = 0x0300;   // DMA2 channel is connected to (ADC1BUF0)
    VMEAS_DMACON.SIZE   = 0;        // Word data transfer size (not byte)
    VMEAS_DMACON.DIR    = 0;        // Read from peripheral, write to RAM
    VMEAS_DMACON.AMODE  = 0b00;     // Register indirect + post-increment
    VMEAS_DMACON.MODE   = 0b10;     // continuous, with ping-pong mode enabled
    VMEAS_DMAREQ.IRQSEL = 0b00001101;    // ADC1 transfer done triggers DMA transfer

    VMEAS_DMASTAL       = __builtin_dmaoffset(&VoltageDataBufferA);
    VMEAS_DMASTAH       = 0;

    VMEAS_DMASTBL       = __builtin_dmaoffset(&VoltageDataBufferB);
    VMEAS_DMASTBH       = 0;

    VMEAS_DMACNT        = 2;        // A 2 actually means 3 DMA transfers (1 word each)
    VMEAS_DMACON.CHEN   = 1;        // Enable DMA2 channel
}

/*
 * Starts sampling voltages
 */
void VMeas_Start()
{
    VMEAS_ADCON1.ASAM = 1;
    VMeas_Init_Timer();
}

void VMeas_Init_Timer()
{
    VMEAS_TMR_IF     = 0;        // Clear Timer3 iterrupt flag
    VMEAS_TMR_IE     = 0;        // Disable Timer3 interrupt
    VMEAS_TCONR      = 0;        // Resets timer
    VMEAS_TCON.TCS   = 0;        // Use internal clock
    VMEAS_TCON.TCKPS = 0b00;     // 1:1 clock precaler
    VMEAS_PR         = 15360;    // 1/60MHz * 15360 = 256us clock period
    VMEAS_TCON.TON   = 1;        // Enable Timer3 and start the counter

}

VoltageData* VMeas_Get_Current_Data()
{
    return (VMEAS_PPST == 0) ? &VoltageDataBufferA : &VoltageDataBufferB;
}

/*
 * Returns filtered voltage measurements if more than a specified number of samples
 * have been filtered. Otherwise, it will return the actual unfiltered measurement
 */
VoltageData* VMeas_Get_Current_Filt_Data()
{
    return &VMeasCurrentFiltData;
}

void VMeas_Init_Filter()
{
    FIRStructInit(&VMeasBattLowPassFilter, LPF_5_1_25_50_SIZE, LPF_5_1_25_50, 0xFF00, VMeasBattFilterDelay);
    FIRDelayInit(&VMeasBattLowPassFilter);
}

void VMeas_Filter_Current_Sample()
{
    VoltageData vd = *(VMeas_Get_Current_Data());
    FIR(1, &(VMeasCurrentFiltData.V_Batt), &(vd.V_Batt), &VMeasBattLowPassFilter);
}