/* 
 *  task.c
 *  
 *  Authors: Peter Li, Lulu Li (no relations)
 *
 *  This is the entry point and basically the entire dev
 *  file for the Piccollo2AMC project
 *
 *  Objectives of this project are:
 *
 *  1) to use the ADC and sample encoder and tachometer values for each of
 *   the Quanser SRV02 motor that is provided for this course
 *
 *  2) To implement a PI control with rate feedback to Control the position of
 *  each motor
 *
 *  3) idunno draw a picture or smth
 *
 *  4) Further stretch goals such as:
 *
 *      a. Feed forward control
 *      b. SensorTalk Communications to remotely control the motor
 *      c. Further stretch goals as we come up with them (i seriously dont think well hit
 *      this step)
 *
 *
 *  This project should be completed NLT Wednesday the 6th of december.
 *
 */

 /* Branch Specific Comments: (DELETE THESE BEFORE MERGE)
 *
 * This branch of task.c This is the development branch for reading the encoder
 *
 * tachometer sensitivity for motor in 7720: 1.5mV/rmp
 *
 * gear ratio: 14
 *
 * tachometer gain [degree/sec] = 1000mVx(rpm/1.5mV)x(360/rotation)x(1min/60sec)x(1/14)
 *
 * Tasks are:
 *
 * 1) To read from the 4 defined GPIOS to develop the system to read 2 pairs of 2 ch
 * encoders
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
#include "plot_sidewind.h"
#include "Library/DSP2802x_Device.h"


// one Feedback Control loop has to complete within this many cycles
// 80 000 cycles is 0.001 seconds
#define CPU_CYCLES_PER_TICK 80000

extern const Semaphore_Handle xDataAvailable;
extern const Semaphore_Handle yDataAvailable;
extern const Swi_Handle xVelProcSwi;
extern const Swi_Handle yVelProcSwi;

// Updated by encoderISR triggers at any time on rising and falling edge
// Highest priority
static volatile int32_t xPos = 0;
static volatile int32_t yPos = 0;

// updated by ADC SWI
static volatile int32_t xVel = 0;
static volatile int32_t yVel = 0;

// updated every CPU_CYCLES_PER_TICK by feedback

#define XVELOFFSET  50
#define YVELOFFSET (0-15)
#define X_OUTPUT 0
#define Y_OUTPUT 1
#define Q_VALUE 16

#define ENCODERCALIB 90 // 360/2048 ticks per rotation represented in q9
#define ENCODERCALIB_Q 9
#define TACHOCALIB 714 // = .6975 Q9
#define TACHOCALIB_Q 9 // = .6975 Q9
#define VOLTAGECALIB_Q
#define VOLTAGEOFFSET_Q

static volatile int32_t voltage[2] = {2048, 2048};

// Updated whenever the draw task needs to
static volatile int32_t xPosRef = 0;
static volatile int32_t yPosRef = 0;
static uint16_t plotting = 1;

// Initial values
#define XPOSINIT -30
#define YPOSINIT -30
#define XPOSREFINIT -30
#define YPOSREFINIT -30
#define PLOTINIT 0

/*
 *  ======== main ========
 */

Int main()
{
    DeviceInit();
    uint32_t shiftval = -30;
    xPos = shiftval << 16;
    yPos = shiftval << 16;

    //initial starting values
    xPos = XPOSINIT;
    xPosRef = XPOSREFINIT;
    yPos = YPOSINIT;
    yPos = YPOSREFINIT;
    plotting = PLOTINIT;

    BIOS_start(); /* does not return */
    return (0);
}


/* Backward, Stop, Forward*/
// per tick in Q16 converted to 360 degrees per rotation
int32_t directions[] = {11520, -11520, -11520, 11520};

Void StepNextPointFxn(){

}

Void xEncISR(Void)
{
    uint16_t xMask;
    xMask =  (GpioDataRegs.GPADAT.bit.GPIO5 << 1) + GpioDataRegs.GPADAT.bit.GPIO4;
    xPos += directions[xMask];
}

// Pins assigned for yMotor are:
// J6.3 = x and J6.4 = y
Void yEncISR(Void)
{
    uint16_t yMask;
    yMask = (GpioDataRegs.GPADAT.bit.GPIO1 << 1) + GpioDataRegs.GPADAT.bit.GPIO0;
    yPos += directions[yMask];
}

uint16_t timeElapsedms_5 = 0;
Void timerISR(Void){
    // Every step, output to the encoder
    static uint16_t xOrY = X_OUTPUT;
    AdcRegs.ADCSOCFRC1.all = 0x3;
    GpioDataRegs.GPATOGGLE.all = 0xC;
    xOrY ^= 1;
    SpiaRegs.SPITXBUF = voltage[xOrY];
    timeElapsedms_5 += 1;

}
#define F_TAPS 8
int16_t xVelRaw[F_TAPS] = {0};
int16_t yVelRaw[F_TAPS] = {0};
// these are moving average filters
Void xVelISR (Void){
    static uint16_t i = 0;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    //if(plotting) // if swi is never posted then the PID function is permanently blocked
        Swi_post(xVelProcSwi);
    xVelRaw[i] = AdcResult.ADCRESULT0 - 2048 + XVELOFFSET;
    i = (i + 1) & 7;
}

Void xVelProcFxn(Void){
    int i;
    int32_t cVel = 0;
    for (i = 0; i < F_TAPS; i++)
        cVel += xVelRaw[i];
    xVel = ((cVel <<11) * TACHOCALIB);
    xVel >>= TACHOCALIB_Q;
    Semaphore_post(xDataAvailable);
}

Void yVelISR(Void){
    static uint16_t i = 0;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
    //if(plotting)
        Swi_post(yVelProcSwi);
    yVelRaw[i] = AdcResult.ADCRESULT1 - 2048 + YVELOFFSET;
    i = (i + 1) & 7;
}

Void yVelProcFxn(Void){
    int i;
    int32_t cVel = 0;
    for (i = 0; i < F_TAPS; i++)
        cVel += yVelRaw[i];
    yVel = ((cVel <<11) * TACHOCALIB);
    yVel >>= TACHOCALIB_Q;
    Semaphore_post(yDataAvailable);
}
/*
 *  ======== Feedback Control Function ========
 * Process the implemented PID control loop SWI
 * triggers once every 0.001s
 */
#define X_KD 123 // 0.00188 in q16
#define X_KP 77 // 0.15 in q9

Void xFeedbackControlFxn(Void)
{
    int32_t cerr;
    while (1)
    {
        Semaphore_pend(xDataAvailable, BIOS_WAIT_FOREVER);
        cerr = xPosRef - xPos;
        cerr = ((cerr * X_KP)>>9) - ((X_KD * xVel)>>16);
        cerr = (cerr * 256); // fix output scale
        cerr = (cerr >> 16) + 2048; // fix output offset
        voltage[X_OUTPUT] = cerr;//(err >> Q_VALUE);
    }
}

#define Y_KD 223 // 0.003412 in q16
#define Y_KP 76 // 0.148 in q9

Void yFeedbackControlFxn(Void)
{
    int32_t cerr;
    while (1)
    {
        Semaphore_pend(yDataAvailable, BIOS_WAIT_FOREVER);
        cerr = yPosRef - yPos;
        cerr = ((cerr * Y_KP)>>9) - ((Y_KD * xVel)>>16);
        cerr = (cerr * 256); // fix output scale
        cerr = (cerr >> 16) + 2048; // fix output offset
        voltage[Y_OUTPUT] = cerr;//(err >> Q_VALUE);

    }
}

#ifdef __P2AMC_MODE_DEBUG
static volatile uint32_t idleTicks = 0;
#endif


// triggers once every .10 seconds, steps voltage reference to the next position
Void StepNextPointTriggerFxn(Void){

    static uint16_t currentstep = 0;
    if(plotting){
        xPosRef = xPlots[currentstep] << 16;
        yPosRef = yPlots[currentstep] << 16;
        currentstep += 1;
        plotting = currentstep < NVALS ? 1 : 0;
    }
}

Void Idle(void)
{
    while (1)
    {
#ifdef __P2AMC_MODE_DEBUG
        idleTicks +=1;
#endif
        if(!plotting)
            xPos; //  TODO enter into sleepmode here
    }
}
