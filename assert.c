#include "assert.h"
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"

// Candidate for my assert Module
void
spinDelayUs(uint32_t us)
{
    while(us--)
    {
        __asm("    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n");
    }
}

void
spinDelayMs(uint32_t ms)
{
    while(ms--)
    {
        spinDelayUs(1000);
    }
}

void
__error__(const char * filename, int line)
{
    _assert_failed("__error__(file,line) called from TivaDriverLib", filename, line);
}


void
_assert_failed (const char *assertion, const char *file, unsigned int line)
{
    // Not alot that we can do at the current time so simply blink the
    // LED rapidly
    //
    // Normally an IO would display:
    //   Assertion failed: expression, file filename, line line number

    // Denergize any outputs

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

    //
    // Do a dummy read to insert a few cycles after enabling the peripheral.
    //
    uint32_t ui32Loop = SYSCTL_RCGC2_R;

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIO_PORTF_DIR_R = (1<<1) | (1<<2) | (1<<3);
    GPIO_PORTF_DEN_R = (1<<1) | (1<<2) | (1<<3);

    // turn off all the LED's
    //
    GPIO_PORTF_DATA_R &= ~((1<<2) | (1<<1) | (1<<3));

    #ifdef USB_SERIAL_OUTPUT
      UARTprintf("Assertion Failed: %s at %s::%d\n",assertion, file, line);
    #endif
    taskENTER_CRITICAL();
    taskDISABLE_INTERRUPTS();

    //
    // Loop forever.
    //
    ui32Loop = 700;
    while(1)
    {
        //
        // Flash Fast the LED.
        //
        int cnt=((ui32Loop/2) * (ui32Loop/13) );
        while(cnt--)
        {
            // This block takes about 13.28us
            GPIO_PORTF_DATA_R |= (1<<2);
            spinDelayUs(10);
            GPIO_PORTF_DATA_R &= ~(1<<2);
            spinDelayUs(26);
        }

        cnt=((ui32Loop/2) * (ui32Loop/13) );
        while(cnt--)
        {
            // This block takes about 13.28us
            GPIO_PORTF_DATA_R |= (1<<1);
            spinDelayUs(2);
            GPIO_PORTF_DATA_R &= ~(1<<1);
            spinDelayUs(34);
        }
    }
}


void vApplicationMallocFailedHook( void )
{
	taskDISABLE_INTERRUPTS();
    assert(0);
	for( ;; );
}


void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	taskDISABLE_INTERRUPTS();
    assert(0);
	for( ;; );
}
