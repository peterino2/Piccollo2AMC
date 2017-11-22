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

/**
 * Branch Specific Comments: (DELETE THESE BEFORE MERGE)
 *
 * This branch of task.c This is the development branch for reading the encoder
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

extern const Semaphore_Handle mySem;
extern const Semaphore_Handle daveSem;


// Updated by encoderISR triggers at any time on rising and falling edge
// Highest priority
volatile static uint32_t xPos;
volatile static uint32_t yPos;

// updated by ADC SWI
volatile static uint32_t xVel;
volatile static uint32_t yVel;

// updated every 0.001s by feedback swi
volatile static uint32_t xVoltage;
volatile static uint32_t yVoltage;

// Updated whenever the draw task needs to
volatile static uint32_t xPosRef;
volatile static uint32_t yPosRef;

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
        1, -1, -1, 1, 1, -1, -1, 1 };

/* Backward, Stop, Forward*/
const int16_t encDirCodes[3] = { 0, 8, 4 };
Void xEncISR(Void)
{
    static int previous = 8;
    uint16_t delta;
    delta = encDeltas[previous|( GpioDataRegs.GPADAT.all >> 28 ) & 0x3];
    previous = encDirCodes[delta + 1];
    xPos += delta;
}

Void yEncISR(Void)
{

}


/*
 *  ======== Feedback Control Function ========
 * Process the implemented PID control loop SWI
 * triggers once every 0.001s
 */
Void FeedbackControlFxn(Void)
{
    int32_t kd; // Rate feedback gain Not used in PI control
    int32_t kp; // Proportional gain
    int32_t ki; // integral gain

    while (1)
    {
    }
}

Void xVelISR (Void){

}
Void yVelISR(Void){

}

Void Idle(void)
{
    while (1)
    {
    }
}
