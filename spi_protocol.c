/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

/* Optional includes for USB serial output */
#ifdef USB_SERIAL_OUTPUT
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#endif

/*local includes*/
#include "assert.h"

// Included for GPIO_O_LOCK and others
#include "inc/hw_gpio.h" //for macro declaration of GPIO_O_LOCK and GPIO_O_CR
#include <inc/hw_types.h>

#include "timers.h"
#include "timing_position.h"


#define SPEED_TEST 0


#define LED_R (1<<1)
#define LED_G (1<<3)
#define LED_B (1<<2)
#define SW1   (1<<4)
#define SW2   (1<<0)
#define PHA (1<<2)
#define PHB (1<<3)

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))
#define pdTICKSTOMS( xTicks ) ((xTicks * 1000 ) / configTICK_RATE_HZ )

// static SemaphoreHandle_t _semPhB = NULL;

uint32_t SystemCoreClock;
uint16_t RPMs[10];
uint8_t RPM_pointer = 0;


#ifdef USB_SERIAL_OUTPUT

//*****************************************************************************
//
//: Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************


static void
_configureUART(void)
{
    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

#endif


static void
_interruptHandlerPortE(void)
{
    uint32_t mask = GPIOIntStatus(GPIO_PORTE_BASE, 1);
    if ((mask & PHA) || (mask & PHB))
    {
      uint8_t phVal, phaVal, phbVal;
      phVal = GPIOPinRead(GPIO_PORTE_BASE, (PHA|PHB));
      phaVal = (phVal & PHA) >= 1?1:0;
      phbVal = (phVal & PHB) >= 1?1:0;

      update_position(phaVal, phbVal);
      insert_time((UINT32_MAX - TimerValueGet(TIMER0_BASE, TIMER_A)));
    }
    GPIOIntClear(GPIO_PORTE_BASE, mask);
}

static void
_setupHardware(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));

    GPIO_PORTE_CR_R = PHA | PHB;
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, (PHA|PHB));
    GPIOIntRegister(GPIO_PORTE_BASE, _interruptHandlerPortE);
    GPIOIntTypeSet(GPIO_PORTE_BASE, (PHA|PHB), GPIO_BOTH_EDGES);

    GPIOPadConfigSet(GPIO_PORTE_BASE, (PHA|PHB), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

}

static void
_setupTimer(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  spinDelayMs(10);
  TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  TimerLoadSet(TIMER0_BASE, TIMER_A, UINT32_MAX);
  TimerEnable(TIMER0_BASE, TIMER_A);
}

static void
_heartbeat( void *notUsed )
{
   timing_init(GPIOPinRead(GPIO_PORTE_BASE, PHA)  & PHA, GPIOPinRead(GPIO_PORTE_BASE, PHB)  & PHB);

   uint32_t green500ms = 500; // 1 second for on off
   uint32_t ledOn = 1;


   while(true)
   {
     vTaskDelay(green500ms / portTICK_RATE_MS);
     LED(LED_G, ledOn);
     ledOn = !ledOn;
    }
}

void _rpm_update(void * notUsed)
{
    while(1) {
      RPMs[RPM_pointer] = motor_get_rpm();
      RPM_pointer = (RPM_pointer + 1) % 10;
      vTaskDelay(50 / portTICK_RATE_MS);
    }
}

uint16_t rpm_average(void)
{
  uint32_t sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += RPMs[i];
  }
  return (sum/10);
}

void
_idle_print_out (void * notUsed)
{
  GPIOIntEnable(GPIO_PORTE_BASE, (PHA|PHB));
  while(1) {
    while(1) {
      #ifdef USB_SERIAL_OUTPUT
        UARTprintf("Position: %d Average RPM: %d\n", motor_get_position(), rpm_average());
      #endif
      vTaskDelay(500 / portTICK_RATE_MS);
    }
  }
}

int main( void )
{
    _setupHardware();
    #ifdef USB_SERIAL_OUTPUT
    	void spinDelayMs(uint32_t ms);
    	_configureUART();
      _setupTimer();
    	spinDelayMs(1000);  // Allow UART to setup

    	UARTprintf("\n\n\nHello from main()\n");
    #endif

    // while(1) {
    //   uint32_t a = TimerValueGet(TIMER0_BASE, TIMER_A);
    //   spinDelayMs(1000);
    //   uint32_t b = TimerValueGet(TIMER0_BASE, TIMER_A);
    //   UARTprintf("%i\n", a-b);
    //
    // }

    xTaskCreate(_rpm_update,
                "update_rpm",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL );

    xTaskCreate(_heartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,  // higher numbers are higher priority..
                NULL );

    xTaskCreate(_idle_print_out,
                "printout",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL );


    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
