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
volatile static int16_t xPos = 0;
volatile static int16_t yPos = 0;

// updated by ADC SWI
volatile static int16_t xVel = 0;
volatile static int16_t yVel = 0;

// updated every CPU_CYCLES_PER_TICK by feedback
volatile static int16_t xVoltage = 0;
volatile static int16_t yVoltage = 0;

// Updated whenever the draw task needs to
volatile static int16_t xPosRef = 0;
volatile static int16_t yPosRef = 0;

/* Counter incremented by Interrupt*/
volatile UInt tickCount = 0;

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


/*
 * encDirections is a lookup table that tells you which direction
 * the encoder is moving
 * */
const int16_t encDeltas[16] = {
        // If previous movement was negative
        0, -1, -1, 0,
        // If previous movement was positive
        1, 0, 0, 1,
        // If previous movement was neither
        1, -1, -1, 1, 1, -1, -1, 1
};

/* Backward, Stop, Forward*/
const uint16_t encDirCodes[3] = { 0, 8, 4 };

#define BACKWARD 0
#define STOP 1
#define FORWARD 2


Void xEncISR(Void)
{
    static uint16_t previous = 8;
    int16_t delta, mask;
    DelayUs(1000);
    mask = ( (GpioDataRegs.GPADAT.bit.GPIO18 << 1) | GpioDataRegs.GPADAT.bit.GPIO29) & 0x3;
    delta = encDeltas[previous|mask];
    previous = encDirCodes[delta + 1];
    xPos += delta;
}

// Pins assigned for yMotor are:
// J6.1 = A and J6.2 = Bn
Void yEncISR(Void)
{
    static int previous = 8;
    uint16_t delta;
    delta = encDeltas[previous|( GpioDataRegs.GPADAT.all ) & 0x3];
    previous = encDirCodes[delta + 1];
    yPos += delta;
}

Void timerISR(Void){
    AdcRegs.ADCSOCFRC1.all = 0x3;
}

Void xVelISR (Void){
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    xVel = AdcResult.ADCRESULT0;
//    Semaphore_post(xDataAvailable);
}

Void yVelISR(Void){
    AdcRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
    yVel = AdcResult.ADCRESULT1;
//    Semaphore_post(yDataAvailable);
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
