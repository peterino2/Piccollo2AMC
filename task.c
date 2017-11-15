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
 * Branch Specific Comments:
 *
 * This branch of task.c
 */
#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Semaphore handle defined in task.cfg */
extern const Semaphore_Handle mySem;
extern const Semaphore_Handle daveSem;

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

    /* 
     * Start BIOS.
     * Begins task scheduling.
     */
    BIOS_start();    /* does not return */
    return(0);
}

/*
 *  ======== myTickFxn ========
 *  Timer ISR function that posts a Swi to perform 
 *  the non-realtime service functions.
 */
Void myTickFxn(UArg arg) 
{
    tickCount += 1;    /* increment the counter */

    /* every 10 timer interrupts post the semaphore */
    if ((tickCount % 10) == 0) {
        Semaphore_post(mySem);
    }
}

/*
 *  ======== myTaskFxn ========
 *  Task function that pends on a semaphore until 10 ticks have
 *  expired.
 */
Void myTaskFxn(Void) 
{
	int i = 3;
    /*
     * Do this forever
     */
    while (TRUE) {
        /* 
         * Pend on "mySemaphore" until the timer ISR says 
         * its time to do something.
         */ 
        Semaphore_pend(mySem, BIOS_WAIT_FOREVER);
        /*
         * Print the current value of tickCount to a log buffer. 
         */
        if(--i == 0){
        	Semaphore_post(daveSem);
        	Log_info1("10 ticks. Tick Count = %d\n", tickCount);
        }
    }
}


Void daveTaskFxn(void){
    /*
     * Pend on "mySemaphore" until the timer ISR says
     * its time to do something.
     */
	while(1){
		Semaphore_pend(daveSem, BIOS_WAIT_FOREVER);
		/*
		 * Print the current value of tickCount to a log buffer.
		 */
		Log_info1("DAVES HERE! 10 ticks. Tick Count = %d\n", tickCount);
	}
}
