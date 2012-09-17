/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "semphr.h"
#include "queue.h"

/* application includes. */
#include "uip.h"
#include "USB-CDC.h"
#include "uIP_Task.h"
#include "BlockQ.h"
#include "blocktim.h"
#include "flash.h"
#include "QPeek.h"
#include "dynamic.h"
#include "debug_printf.h"

#include <aic/aic.h>
#include <pio/pio.h>
#include <pmc/pmc.h>
#include <pwmc/pwmc.h>
#include <utility/led.h>
#include <utility/trace.h>
/*-----------------------------------------------------------*/

/* Task Priorities */
#define mainBLOCK_Q_PRI           ( tskIDLE_PRIORITY + 1 )
#define mainFLASH_PRI             ( tskIDLE_PRIORITY + 1 )
#define mainGEN_QUEUE_PRI         ( tskIDLE_PRIORITY ) 
#define mainUSB_PRI               ( tskIDLE_PRIORITY + 1 )
#define mainUIP_PRI               ( tskIDLE_PRIORITY + 1 )

/* Stack Sizes */
#define mainUSB_STACK             ( 200 )
#define mainUIP_STACK             ( 200 )

/* Servo Constants */
#define CHANNEL_PWM_SERVO1                  2
#define CHANNEL_PWM_SERVO2                  3
#define SERVO_PWM_FREQUENCY                 50
#define SERVO_DUTY_CYCLE                    2000
#define MAX_DEGREES                         254  
#define MIN_DEGREES                         52  
#define MAX_DUTY_CYCLE                      100
#define MIN_DUTY_CYCLE                      0

/* Imposes the maximum and minimum duty cycle rated for the servo's */ 
#define vImposeServoLimits(PAN,TILT)                \
    do {                                            \
    if(PAN > MAX_DEGREES) PAN = MAX_DEGREES;        \
    if(PAN < MIN_DEGREES) PAN = MIN_DEGREES;        \
    if(TILT > MAX_DEGREES) TILT = MAX_DEGREES;      \
    if(TILT < MIN_DEGREES) TILT = MIN_DEGREES;      \
    }while(0)

/*-----------------------------------------------------------*/
void vPBISR_Handler( void ) __attribute__((naked));
void vPBISR_Wrapper( void ) __attribute__((naked));

static void prvSetupHardware( void );
void vApplicationIdleHook( void );

const Pin PIN_SET[] = {
    PIN_PUSHBUTTON_1,
    PIN_PWMC_PWM2,
    PIN_PWMC_PWM3,
    PIN_LASER,
};

/*-----------------------------------------------------------*/
int main( void )
{
    /* Hardware Setup */
    /*-----------------------------------------------------------*/
    prvSetupHardware();
    PIO_Configure(PIN_SET, PIO_LISTSIZE(PIN_SET));

    /* System Tasks */
    /*-----------------------------------------------------------*/
    xTaskCreate( vUSBCDCTask, ( signed char * ) "USB", mainUSB_STACK, NULL, mainUSB_PRI, NULL );
    /* Start the standard tasks. */
    vStartBlockingQueueTasks( mainBLOCK_Q_PRI );
    vCreateBlockTimeTasks();
    vStartGenericQueueTasks( mainGEN_QUEUE_PRI );
    vStartQueuePeekTasks();   
    vStartDynamicPriorityTasks();

    /* User Tasks */
    /*-----------------------------------------------------------*/


    vTaskStartScheduler();

    /* We should never get here as control is now taken by the scheduler. */
    return 0;
}

/*-----------------------------------------------------------*/
/* Hardware Initialisation */
static void prvSetupHardware( void )
{
    portDISABLE_INTERRUPTS();

    /* When using the JTAG debugger the hardware is not always initialised to
    the correct default state.  This line just ensures that this does not
    cause all interrupts to be masked at the start. */
    AT91C_BASE_AIC->AIC_EOICR = 0;

    /* Most setup is performed by the low level init function called from the
    startup asm file. */

    /* Enable the peripheral clock. */
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOB;
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_EMAC;

    /* Initialise the LED outputs for use by the demo application tasks. */
    LED_Configure(0);
}

/* may be useful*/
void vApplicationTickHook( void ){
    
    return;
}

/*-----------------------------------------------------------*/
/* Idle Application Task */
void vApplicationIdleHook( void )
{
    static portTickType xLastTx = 0;

    /* The idle hook simply prints the idle tick count */
    if( ( xTaskGetTickCount() - xLastTx ) > ( 1000 / portTICK_RATE_MS ) )
    {
        xLastTx = xTaskGetTickCount();
        LED_Toggle(0);
    }
}
