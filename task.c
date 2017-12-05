/* 
 *  task.c
 *  
 *  Authors: Peter Li, Lulu Li (no relations)
 *
 *  This is the entry point to the project
 *
 *  Objectives of this project are:
 *
 *  1) To use the adc and the sample encoder and tachometer values.
 *  Do this for each of the Quanser SRV02 motors that is provided for this course
 *
 *  2) To implement a proportional with rate feedback controller to control the position
 *  of each motor.
 *
 *  3) Draw a picture from memory
 *
 *  4) Program a demo - draw a tree
 *
 *  Usage:
 *
 *      1. Set the arms such that both arms at a rotation of 0 degrees
 *      2. Press the button associated with GPIO12
 *
 *  (Optional) Compile with __P2AMC_MODE_DEBUG defined in order to enable debug mode
 *  This prevents the motor from entering sleeping and instead allows the controller
 *  to profile its' time spent in the idle function
 *
 *
 */

#define xdc__strict
#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>
#include "Library/Devinit.h"
#include "Library/DSP2802x_Device.h"

#define __P2AMC_MODE_DEBUG // uncomment to enable debug mode

// load the demo file here
// demo file must include
/*
 * #define NVAL <number of vertexes>
 * const int32_t xPlots[NVALS] = {Array of x vertices};
 * const int32_t xPlots[NVALS] = {Array of y vertices};
 * */
#include "christmas_tree.h"


// ---- Rtos Externs ----

extern const Swi_Handle xVelProcSwi; // Posted by xVelISR
extern const Swi_Handle yVelProcSwi; // Posted by yVelISR

extern const Semaphore_Handle xDataAvailable; // Posted upon completion of xVelProcSwi
extern const Semaphore_Handle yDataAvailable; // Posted upon completion of yVelProcSwi


// ---- Program State Machines ----


static enum{
    stopped, plotting
} state = stopped;

typedef enum {
    out_x, out_y
} Outputs;


// ---- Calibration and numerical constants ----



#define Q_VALUE 16 // Q value base used for representing internal operations

// -- Encoder Calibration constants --

// per tick in Q16 converted to 360 degrees per rotation
// This is used with the GPIO read bits as a lookup table
static const int32_t directions[] = {11520, -11520, -11520, 11520};

#define ENCODERCALIB 90 // 360/2048 ticks per rotation represented in q9
#define ENCODERCALIB_Q 9

// -- ADC and DAC Constants --
// This is a single value that represents the conversion gain from raw adc
// units to degrees per second coming from the tachometer
#define TACHOCALIB 714 // = 0.6975 in Q9
#define TACHOCALIB_Q 9 // Q Value for the calibration constant


// The tachometers have an error that cause them to output an offset
// voltage when no movement is happening.
// These constants should be added to the ADC value to correct for these errors

#define XVELOFFSET  50      // X adc offset
#define YVELOFFSET (0-15)   // Y adc offset

// This is the ADC mid point, our level shifting circuit shifts 0v as
// 1.65 volts which the ADC represents as 2048
#define ADCMIDPOINT 2048

// This is the DAC mid point, our level shifting circuit shifts 1.65v as
// 0v which our DAC requires a digital read value of 2048
#define DACMIDPOINT 2048


// ---- Controller Constants ----

#define X_KD 123    // 0.00188 in q16
#define X_KD_Q 16   // Q value for X_KD calculations

#define X_KP 140    // 0.15 in Q9
#define X_KP_Q 9    // Q value for X_KP calculations

#define Y_KD 223    // 0.003412 in q16
#define Y_KD_Q 16   // Q value for Y_KD calculations

#define Y_KP 120    // 0.148*2 in Q9
#define Y_KP_Q 9    // Q value for Y_KP calculations

#define OUTPUT_SCALE 256// Converts 0-16 Q16 to 0->4096 Q0


// ---- Controller Variables ----

// Updated by encoderISR
// These will be updated at any rising or falling edge on any of the
// Four encoder variables
// stored in degrees with a Q value of Q_VALUE
static volatile int32_t xPos = 0;
static volatile int32_t yPos = 0;

// Updated by nVelProcFxn Swi, result of a moving average filter
static volatile int32_t xVel = 0; // X velocity
static volatile int32_t yVel = 0; // Y velocity

// Finite impulse response filter definition
#define F_TAPS 8                // Must be a power of 2
#define F_TAPS_Q 4              // Equivalent Q value of summing the taps
#define F_INDX_MASK (F_TAPS-1)   // Filter Bit mask

// Updated by xVelISR
// Circular buffers for calculating x and y velocity
static int16_t xVelRaw[F_TAPS] = {0};
static int16_t yVelRaw[F_TAPS] = {0};

// Updated whenever the draw task needs to
static volatile int32_t xPosRef = 0; // x motor Reference position
static volatile int32_t yPosRef = 0; // y motor Reference Position


// ---- SPI output buffers ----
// Output buffers for the x and y voltages
static volatile int32_t voltage[2] = {DACMIDPOINT, DACMIDPOINT};


// ---- Entry Point ----
Int main()
{
    DeviceInit(); // Initialize the device

    /* << E < E < E N T E R T H E B I O S > S > S >>> */
    /*|*/                                          /*|*/
    /*|*/               BIOS_start();              /*|*/
    /*|*/                                          /*|*/
    /* << E < E < E N T E R T H E B I O S > S > S >>> */
    return (0);
}


/*
 * X and Y encoder ISR
 *
 * Updates the current position of the rotary arm based on the pulses generated
 * by the encoders
 *
 * Each encoder has a trigger line that is both rising and falling edge
 *  - Gives us 2x Quadrature resulting in 2048 pulses per rotation
 *
 * Read both the GPIO pins assigned to the encoder motor
 *
 * If they are equal, then increment the motor position,
 * if they are not equal then decrement the motor position
 *
 * The lookup table directions is used to increment and decrement the current
 * positions, this avoids branching.
 * */

// Triggered by XINT1, a GPIO interrupt pulsed for both rising and falling edge
Void xEncISR(Void)
{
    // Generate an index
    uint16_t index;
    index =  (GpioDataRegs.GPADAT.bit.GPIO5 << 1) + GpioDataRegs.GPADAT.bit.GPIO4;

    // index through lookup table and increment/decrement the xPosition
    xPos += directions[index];
}

// Triggered by XINT1, a GPIO interrupt pulsed for both rising and falling edge
Void yEncISR(Void)
{

    // Generate an index
    uint16_t index;
    index = (GpioDataRegs.GPADAT.bit.GPIO1 << 1) + GpioDataRegs.GPADAT.bit.GPIO0;

    // index through lookup table and increment/decrement the xPosition
    yPos += directions[index];
}


/*
 * ---- pidStepISR ----
 *
 * Start the next step of the pid loop sample
 *
 * Do this by starting the ADC conversions and toggling the GPIO outputs
 *
 * This causes our PID loop to have a resolution of 0.001ms our actual
 * outputs voltages will have a step size of 0.002ms
 *
 * */

#ifdef __P2AMC_MODE_DEBUG
uint16_t timeElapsedms = 0; // used in debug mode to count time elapsed
#endif

Void pidStepISR(Void){
    // Every step, output to the encoder
    static Outputs xOrY = out_x;
    AdcRegs.ADCSOCFRC1.all = 0x3;       // Start both pid loops
    GpioDataRegs.GPATOGGLE.all = 0xC;   // Toggle the two GPIO 3 and 4
    xOrY ^= 1;                          // toggle which buffer we are transmitting
    SpiaRegs.SPITXBUF = voltage[xOrY];  // Output corresponding buffer for the adc

#ifdef __P2AMC_MODE_DEBUG
    timeElapsedms += 1; // increment the time elapsed for use time profiling
#endif

}


/* ---- xVelISR ----
 *
 * x motor adc velocity ISR
 *
 * Collects adc value and loads it into the xVelRaw circular buffer so it can be
 * digitally filtered.
 *
 * it then posts the swi for the digital filter
 *
 * */

Void xVelISR (Void){
    static uint16_t i = 0;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Clear flags
    if(state == plotting)
        Swi_post(xVelProcSwi);
    // Get adc result and mask the index for the circular buffer
    xVelRaw[i] = AdcResult.ADCRESULT0 - ADCMIDPOINT + XVELOFFSET;
    i = (i + 1) & F_INDX_MASK;
}


/* ---- yVelISR ----
 *
 * y motor adc velocity ISR
 *
 * Collects adc value and loads it into the yVelRaw circular buffer so it can be
 * digitally filtered.
 *
 * it then posts the swi for the digital filter
 *
 * */

Void yVelISR(Void){
    static uint16_t i = 0;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; // Clear flags
    if(state == plotting)
        Swi_post(yVelProcSwi);

    // Get adc result and mask the index for the circular buffer
    yVelRaw[i] = AdcResult.ADCRESULT1 - ADCMIDPOINT + YVELOFFSET;
    i = (i + 1) & F_INDX_MASK;
}

/*
 * ---- xVelProcFxn ----
 *
 * Swi for converting and filtering the raw adc data into a single adc
 * value in degrees per second
 * */

Void xVelProcFxn(Void){
    int i;
    int32_t cVel = 0;
    // Accumulate
    for (i = 0; i < F_TAPS; i++)
        cVel += xVelRaw[i];

    // Apply moving average filter and convert to degrees per second
    cVel = ((cVel << (Q_VALUE - F_TAPS_Q)) * TACHOCALIB);

    // Qualify value and store as the current velocity
    xVel =  cVel >> TACHOCALIB_Q;

    // signal data is ready for pid loop
    Semaphore_post(xDataAvailable);
}



/*
 * ---- xVelProcFxn ----
 *
 * Swi for converting and filtering the raw adc data into a single adc
 * value in degrees per second
 * */

Void yVelProcFxn(Void){
    int i;
    int32_t cVel = 0;
    // Accumulate
    for (i = 0; i < F_TAPS; i++)
        cVel += yVelRaw[i];

    // Apply moving average filter and convert to degrees per second
    yVel = ((cVel << (Q_VALUE - F_TAPS_Q)) * TACHOCALIB);

    // Qualify value and store as the current velocity
    yVel >>= TACHOCALIB_Q;

    // signal data is ready for pid loop
    Semaphore_post(yDataAvailable);
}
/*
 *  ======== Feedback Control Function ========
 * Process the implemented PID control loop SWI
 * triggers once every 0.001s
 *
 *                                                       +-----+
 *          +       +------+     +         +-----+       |  1  |
 *  Ref----->O+-----+ KP   +-----+>O+------>motor+---+-->+ --- +----+-> Encoder
 *          -^      +------+      -^       +-----+   |   |  s  |    |
 *           |                     |                 |   +-----+    |
 *           |                     |       +-----+   |              |
 *           |                     ^-------+ KD  <---+tachometer    |
 *           |                             +-----+                  |
 *           |                                                      |
 *           |                                                      |
 *           ^------------------------------------------------------+
 *
 *              Listing 1 : Proportional with rate feedback controller
 *
 */

Void xFeedbackControlFxn(Void)
{
    int32_t cerr;
    while (1)
    {
        // see listing 1 for what kind of feedback controller this implements
        Semaphore_pend(xDataAvailable, BIOS_WAIT_FOREVER);
        cerr = xPosRef - xPos;
        cerr = ((cerr * X_KP)>>X_KP_Q) - ((X_KD * xVel)>>X_KD_Q);
        cerr = (cerr * OUTPUT_SCALE); // fix output scale
        cerr = (cerr >> Q_VALUE) + DACMIDPOINT; // fix output offset
        voltage[out_x] = cerr;
    }
}

Void yFeedbackControlFxn(Void)
{
    int32_t cerr;
    while (1)
    {
        // see listing 1 for what kind of feedback controller this implements
        Semaphore_pend(yDataAvailable, BIOS_WAIT_FOREVER);
        cerr = yPosRef - yPos;
        cerr = ((cerr * Y_KP)>>X_KP_Q) - ((Y_KD * xVel)>>X_KD_Q);
        cerr = (cerr * OUTPUT_SCALE); // fix output scale
        cerr = (cerr >> Q_VALUE) + DACMIDPOINT; // fix output offset
        voltage[out_y] = cerr;

    }
}

#ifdef __P2AMC_MODE_DEBUG
static volatile uint32_t idleTicks = 0;
#endif



/*
 * ---- StepNextPointTriggerFxn ----
 *
 *
 * Timer Triggered ISR that triggers once every .1 seconds.
 *
 * This calculates the next position to be fed to the x and y reference
 *
 * The machine tracks steady ramps much better than it can track large step inputs
 * This function breaks large steps into a series of smooth ramp values
 * */

int32_t xDiff,yDiff; // How far we are from our desired position
static uint16_t currentstep = 0; // The vertex in x and y Plots we are moving to
const int32_t maxStep = 0x30000; // The max size of each discrete step

Void StepNextPointTriggerFxn(Void){
    uint32_t xAtRef = 0;    // have we arrived at our x vertex yet?
    uint32_t yAtRef = 0;    // have we arrived at our y vertex yet?
    int32_t absDiff = 0;    // comparison variable for the absolute value

    if(state){
        // Compare our current reference to the desired vertex position
        xDiff = ((xPlots[currentstep] << Q_VALUE) - xPosRef );

        absDiff = (xDiff >= 0) ? xDiff: -xDiff;

        // Calculate the total distance and move accordingly
        if( absDiff > maxStep ){
            xPosRef += (xDiff >= 0)? maxStep: -maxStep;
        }

        // If we are under the max step range then set our position to the
        // desired position
        else {
            xAtRef = 1;
            xPosRef = xPlots[currentstep] << Q_VALUE;
        }

        // Compare our current reference to the desired vertex position

        yDiff = ( (yPlots[currentstep] << Q_VALUE) - yPosRef );

        absDiff = (yDiff >= 0) ? yDiff: -yDiff;

        // Calculate the total distance and move accordingly
        if( absDiff > maxStep ){
            yPosRef += (yDiff >= 0)? maxStep: -maxStep;
        }

        // If we are under the max step range then set our position to the
        // desired position
        else {
            yAtRef = 1;
            yPosRef = yPlots[currentstep] << Q_VALUE;
        }
        // Step forward only if we have reached our desired vertex
        currentstep += xAtRef && yAtRef;

        // Stop the plotting and pid loops if we have reached the end of our path
        state = currentstep < NVALS ? plotting : stopped;
    }
}

Void Idle(void)
{
    while (1)
    {
#ifdef __P2AMC_MODE_DEBUG
        idleTicks +=1;
#endif
#ifndef __P2AMC_MODE_DEBUG
        if(state == stopped)
            asm(" IDLE");
#endif
    }
}
