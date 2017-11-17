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
/* Semaphore handle defined in task.cfg */
extern const Semaphore_Handle mySem;
extern const Semaphore_Handle daveSem;

/* Per Encoder Pin definitions for Motors X and Y*/
#define X_ENCA_DATA GpioDataRegs.GPADAT.bit.GPIO28 // J1.3 ---> (GPADAT.all >> 28) & 0x3 // Grab bits 28 and 29
#define X_ENCB_DATA GpioDataRegs.GPADAT.bit.GPIO29 // J1.4

#define Y_ENCA_DATA GpioDataRegs.GPADAT.bit.GPIO0  // J6.1 GPB&0x3
#define Y_ENCB_DATA GpioDataRegs.GPADAT.bit.GPIO1  // J6.2

/* Counter incremented by Interrupt*/
volatile UInt tickCount = 0;

/*
 *  ======== main ========
 */

Int main()
{
    /*
     * Print "Hello world" to a log buffer. 
     */
    Log_info0("Hello world\n");
    DeviceInit();
    /* 
     * Start BIOS.
     * Begins task scheduling.
     */
    BIOS_start(); /* does not return */
    return (0);
}

/*
 *  ======== Encoder interrupt functions ========
 */

/* Encoder multiplexing
 *
 * L A B  R
 * 0 0 0 +1
 * 0 0 1 -1
 * 0 1 0 -1
 * 0 1 1 +1
 * 1      1
 * 1      0
 * 1      0
 * 1      1
 * -1      0
 * -1     -1
 * -1     -1
 * -1      0
 *
 * Encode -1 as 0 and +1 as 1 and 0 as b1x\
 *
 * encDirections is a lookup table that tells you which direction
 * the encoder is moving
 * */
const int16_t encDirections[16] = {
// If previous movement was negative
        0, -1, -1, 0,
        // If previous movement was positive
        1, 0, 0, 1,
        // If previous movement was neither
        1, -1, -1, 1, 1, -1, -1, 1
};
/* This constructs the indices for the lookup table
 * */
const int16_t encDirCodes[3] = { 0, 8, 4 };

volatile static uint32_t xPosition = 0;
Void xEncISR(Void)
{
    static int previous = 8;
    uint16_t delta;
    delta = [previous|( GpioDataRegs.GPADAT.all >> 28 ) & 0x3];
    previous = encDirCodes[ delta + 1 ];
    xPosition += delta;
}

Void yEncISR(Void)
{

}

/* Encoder interrupt Function */

/*
 *  ======== Feedback Control Function ========
 * Process the implemented PID control loop
 */
Void FeedbackControlFxn(Void)
{
// kD is not availalbe yet

int32_t kd; // Rate feedback gain Not used in PI control
int32_t kp; // Proportional gain
int32_t ki; // integral gain

/*
 * Do this forever
 */
while (1)
{
}
}

Void Idle(void)
{
/*
 * Pend on "mySemaphore" until the timer ISR says
 * its time to do something.
 */
while (1)
{
}
}
