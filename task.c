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

#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include "Library/Devinit.h"
#include "Library/DSP2802x_Device.h"


// one Feedback Control loop has to complete within this many cycles
// 80 000 cycles is 0.001 seconds
#define CPU_CYCLES_PER_TICK 80000

extern const Semaphore_Handle xDataAvailable = 0;
extern const Semaphore_Handle yDataAvailable = 0;


// Updated by encoderISR triggers at any time on rising and falling edge
// Highest priority
volatile static int32_t xPos = 0;
volatile static int32_t yPos = 0;

// updated by ADC SWI
volatile static int32_t xVel = 0;
volatile static int32_t yVel = 0;

// updated every CPU_CYCLES_PER_TICK by feedback

#define X_OUTPUT 0
#define Y_OUTPUT 1
volatile static int32_t voltage[2] = {2400, 1800}; // approximately 1V

// Updated whenever the draw task needs to
volatile static int32_t xPosRef = 0;
volatile static int32_t yPosRef = 0;

/* Counter incremented by Interrupt*/

/*
 *  ======== main ========
 */

Int main()
{
    // Sample: Log_info0("Hello world\n");

    DeviceInit();

    BIOS_start(); /* does not return */
    return (0);
}


/* Backward, Stop, Forward*/
// Pins assigned for xMotor are:
// J

int16_t directions[] = {1, -1, -1, 1};

Void xEncISR(Void)
{
    uint16_t mask;
    // motor pins on 18 and 29
    mask = (GpioDataRegs.GPADAT.bit.GPIO28 << 1) + GpioDataRegs.GPADAT.bit.GPIO29;
    xPos += directions[mask];
}

//xMotor select: GPIO 0
//yMotor select: GPIO 1

// Pins assigned for yMotor are:
// J6.1 = x and J6.2 = y
Void yEncISR(Void)
{
    uint16_t mask;
    // motor pins on 18 and 29
    mask = (GpioDataRegs.GPADAT.bit.GPIO28 << 1) + GpioDataRegs.GPADAT.bit.GPIO29;
    yPos += directions[mask];
}

uint16_t xOrYSet[] = {8, 4};
uint16_t xOrYClear[] = {4, 8};

Void timerISR(Void){
    // Every step, output to the encoder
    static uint16_t xOrY = X_OUTPUT;
    AdcRegs.ADCSOCFRC1.all = 0x3;
    // Output

    //when motor not running, DAC outputs 0; which gets shifted to -10V
    //so whenever a motor not running; the dac needs to be set to

    //GpioDataRegs.GPASET.all = xOrYSet[xOrY];
    //GpioDataRegs.GPACLEAR.all = xOrYClear[xOrY];

    SpiaRegs.SPITXBUF = voltage[xOrY];
    GpioDataRegs.GPATOGGLE.all = 0x12;
    xOrY ^= 1;
}

Void xVelISR (Void){
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    xVel = AdcResult.ADCRESULT0;
    //Semaphore_post(xDataAvailable);
}

Void yVelISR(Void){
    AdcRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
    yVel = AdcResult.ADCRESULT1;
    //Semaphore_post(yDataAvailable);
}


/*
 *  ======== Feedback Control Function ========
 * Process the implemented PID control loop SWI
 * triggers once every 0.001s
 */
#define X_KD 1
#define X_KP 1
#define X_KI 1
Void xFeedbackControlFxn(Void)
{
    static int16_t integral;
    while (1)
    {
        //Semaphore_pend(xDataAvailable);
        // Process for xVoltage

        //xVoltage = 2457; // approximately 1V
        //GpioDataRegs.GPADAT.bit.GPIO0 = 1; //run xmotor
        //SpiaRegs.SPIDAT = xVoltage;
    }
}

#define Y_KD 1
#define Y_KP 1
#define Y_KI 1

Void yFeedbackControlFxn(Void)
{
    static int16_t integral;
    while (1)
    {
        //Semaphore_pend(yDataAvailable);
        // Process for yVoltage
    }
}


Void Idle(void)
{
    while (1)
    {

    }
}
