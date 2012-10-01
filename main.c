/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "semphr.h"
#include "queue.h"
#include "ff.h"

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
#define SERVO_PRI                 ( tskIDLE_PRIORITY + 1 )
#define PBTASK_PRI                 ( tskIDLE_PRIORITY + 1 )

/* Stack Sizes */
#define mainUSB_STACK             ( 200 )
#define mainUIP_STACK             ( 200 )
#define SERVO_STACK               ( 400 )
#define PBTASK_STACK                  ( 200 )

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

#define vLogPrintf(...) do {                                                \
        portTickType xSecs, xMins, xHours;                                  \
        xSecs = xTaskGetTickCount()/1000;                                   \
        xMins = xSecs/60;                                                   \
        xHours = xMins/60;                                                  \
        f_printf(&Fil, "%d:%d:%d - ", xHours, xMins % 60, xSecs % 60);      \
        f_printf(&Fil, __VA_ARGS__);                                        \
    } while(0)

#define vLogSync() f_sync(&Fil)
#define pPinLaser (&PIN_SET[3])
#define pPinLED0 (&PIN_SET[4])

#define RESET_CTRL { 1 << 30U, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1,0}

void vPBISR_Handler( void ) __attribute__((naked));
void vPBISR_Wrapper( void ) __attribute__((naked));

void prvSetupHardware( void );
void vApplicationIdleHook( void );
void vConfigurePWM(void);

/* Task Prototypes */
void vServoTask(void* pvParameters);
void vPBTask(void* pvParameters);

static const Pin PIN_SET[] = {
    PIN_PUSHBUTTON_1,
    PIN_PWMC_PWM2,
    PIN_PWMC_PWM3,
    PIN_LASER,
    RESET_CTRL,
};

/* SD Card / File System stuff */

static FATFS Fatfs;    /* File system object */
static FIL Fil;        /* File object */
static BYTE Buff[128]; /* File read buffer */

/* PushButton Semaphore */
xSemaphoreHandle xPBSemaphore;

/* Tuio Data Queue */
xQueueHandle xQueueTuioData;

void vLogOpen( void ) {

    FRESULT error;         /* Result code */
    portCHAR pcLogName[14];
    unsigned portCHAR ucVersion = 1;

    f_mount(0, &Fatfs);		/* Register volume work area (never fails) */

    for ( ; ; ) {

        sprintf(pcLogName, "6342_V%d.TXT", ucVersion++); 

        /* returns FR_EXIST if file already exists */ 
        error = f_open(&Fil, pcLogName, FA_WRITE | FA_CREATE_NEW);
        
        if( error == 0 ) break;

    }

    vLogSync();
}

DWORD get_fattime (void)
{
	return	  ((DWORD)(2012 - 1980) << 25)	/* Year = 2012 */
			| ((DWORD)1 << 21)	/* Month = 1 */
			| ((DWORD)1 << 16)	/* Day_m = 1*/
			| ((DWORD)0 << 11)	/* Hour = 0 */
			| ((DWORD)0 << 5)	/* Min = 0 */
			| ((DWORD)0 >> 1);	/* Sec = 0 */
}

int main( void )
{

    /* Setup Data Structures */

    /* Incoming TUIO Data Queue */
    xQueueTuioData =  xQueueCreate(1, sizeof(unsigned int) * 10);

    /* Create binary semaphore  */  
    vSemaphoreCreateBinary( xPBSemaphore );

    /* Setup Log File for writing */
    vLogOpen();
    
    /* Hardware Setup */
    prvSetupHardware();
    vConfigurePWM();
    PIO_Configure(PIN_SET, PIO_LISTSIZE(PIN_SET));

    /* System Tasks */
    xTaskCreate( vUSBCDCTask, ( signed char * ) "USB", mainUSB_STACK, NULL, mainUSB_PRI, NULL );
    /* Start the standard tasks. */
    vStartBlockingQueueTasks( mainBLOCK_Q_PRI );
    vCreateBlockTimeTasks();
    vStartGenericQueueTasks( mainGEN_QUEUE_PRI );
    vStartQueuePeekTasks();   
    vStartDynamicPriorityTasks();

    /* User Tasks */
    // Start the UIP stack task (netduinoplus/uip/uIP_Task.c)
    xTaskCreate(vuIP_Task, (char * ) "uIP_Task", mainUIP_STACK, NULL, mainUIP_PRI, NULL);

    // Start Servo Tracking Task
    xTaskCreate(vServoTask, (char * ) "Servo", SERVO_STACK, NULL, SERVO_PRI, NULL);

    // Start External Interrupt Dispatcher Task
    xTaskCreate(vPBTask, (char * ) "PushButton", PBTASK_STACK, NULL, PBTASK_PRI, NULL);

    vTaskStartScheduler();

    return 0;
}

void vServoTask(void* pvParameters){
    
    unsigned int pxTuioData[10]; 

    portSHORT sPan, sTilt, sPosX, sPosY;
    portFLOAT fPosX, fPosY;

    sPosX = sPosY = 0;    
    fPosX = fPosY = 0.0f;    
    
    sPan = 125;
    sTilt = 220;

    // Establish TCP Connection
    u16_t ripaddr[2];
    uip_ipaddr(&ripaddr,192,168,0,4);
    uip_connect(&ripaddr, htons(3000));

    for( ; ; ) {
 
        PWMC_SetDutyCycle(CHANNEL_PWM_SERVO1, sPan);
        PWMC_SetDutyCycle(CHANNEL_PWM_SERVO2, sTilt);

        vTaskDelay(100);

        if( xQueueReceive( xQueueTuioData, (void*) pxTuioData, 10) ){

            LED_Toggle(0);

            fPosX = *(float *)&pxTuioData[2]; 	//range 0...1
            fPosY = *(float *)&pxTuioData[3]; 	//range 0...1

            sPosX = (portSHORT) ( fPosX * 12.0f ); //range 0...12
            sPosY = (portSHORT) ( fPosY * 10.0f ); //range 0...10

            if( sPosX < 0 || sPosX > 12 ) continue;
            if( sPosY < 0 || sPosY > 10 ) continue;

            sPan  -= ( sPosX - 5 );
            sTilt += ( sPosY - 5 );

            vImposeServoLimits( sPan, sTilt );

            vLogPrintf("[vServoTask] %dx, %dy, %dp, %dt\r\n", sPosX, sPosY, sPan, sTilt);
        
            debug_printf("[vServoTask] %dx, %dy, %dp, %dt\r\n", sPosX, sPosY, sPan, sTilt);

            vLogSync();
        }

    }

}

/* Pushbutton Input Task */
void vPBTask(void *pvParameters) {

    //DO NOT USE PIO_InitializeInterrupts, PIO_ConfigureIt or PIO_EnableIt.

    int i;

    portENTER_CRITICAL();

    //Call vPassPBSemaphore (pbISR.c)
    vPassPBSemaphore(xPBSemaphore);

    //Setup the PIO interrupt and set ISR to point to PB ISR Wrapper.
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
    AT91C_BASE_PIOA->PIO_ISR; 
    AT91C_BASE_PIOA->PIO_IDR = 0xFFFFFFFF;
    AIC_ConfigureIT(AT91C_ID_PIOA, AT91C_AIC_PRIOR_LOWEST, vPBISR_Wrapper);

    //Manually set your PIO IER register
    AT91C_BASE_PIOA->PIO_IER = PIN_SET[0].mask;
    AIC_EnableIT(AT91C_ID_PIOA);

    portEXIT_CRITICAL();

    for ( ; ; ) {

        vTaskDelay(100);

        if(xSemaphoreTake(xPBSemaphore, 10)){

            debug_printf("[vPBTask] xPBSemaphore Taken\r\n");

            for(i = 0; i < 20; i++){

                i % 2 ? PIO_Clear(pPinLaser) : PIO_Set(pPinLaser);

                vTaskDelay(50);
            }
        }

        vTaskDelay(100);
    }
}

/*-----------------------------------------------------------*/
/* Hardware Initialisation */
void prvSetupHardware( void )
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

        // Sync/Flush log data
//        vLogSync();
    }
}
void vConfigurePWM(void)
{
    // Enable PWMC peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC;

    // Set clock A for servo's at 50hz * Duty cycle (samples).
    PWMC_ConfigureClocks(2 * SERVO_PWM_FREQUENCY * SERVO_DUTY_CYCLE, 0, BOARD_MCK);

    // Configure PWMC channel for SERVO (center-aligned, inverted polarity)
    PWMC_ConfigureChannel(CHANNEL_PWM_SERVO1, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(CHANNEL_PWM_SERVO1, SERVO_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_PWM_SERVO1, MIN_DEGREES);
    
    PWMC_ConfigureChannel(CHANNEL_PWM_SERVO2, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(CHANNEL_PWM_SERVO2, SERVO_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_PWM_SERVO2, MIN_DEGREES);
    
    // Enable channel #1 and #2
    PWMC_EnableChannel(CHANNEL_PWM_SERVO1);
    PWMC_EnableChannel(CHANNEL_PWM_SERVO2);

}
