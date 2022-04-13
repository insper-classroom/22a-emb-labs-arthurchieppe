/*
* Example RTOS Atmel Studio
*/

#include <asf.h>
#include "conf_board.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_LED1_STACK_SIZE (1024/sizeof(portSTACK_TYPE))
#define TASK_LED1_PRIORITY (tskIDLE_PRIORITY)

#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)

#define LED1_PIO       PIOA
#define LED1_PIO_ID    ID_PIOA
#define LED1_IDX       0u
#define LED1_IDX_MASK  (1u << LED1_IDX)

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void pin_toggle(Pio *pio, uint32_t mask);
void LED_init(int estado);
static void configure_console(void);

/************************************************************************/
/* RTOS Hooks                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) { }
}

extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) { configASSERT( ( volatile void * ) NULL ); }

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
		pio_clear(pio, mask);
	else
		pio_set(pio,mask);
}

void LED_init(int estado) {
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_IDX_MASK, estado, 0, 0);
};

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	stdio_serial_init(CONF_UART, &uart_serial_options);
	setbuf(stdout, NULL);
}

/************************************************************************/
/* Tasks                                                                */
/************************************************************************/

static void task_monitor(void *pvParameters) {
	static portCHAR szList[256];

	for (;;) {
		printf("--- task ## %u\n", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(1000);
	}
}

static void task_led(void *pvParameters) {
	LED_init(1);
	const TickType_t xDelay = 2000 / portTICK_PERIOD_MS;
	for (;;) {
		pin_toggle(LED_PIO, LED_IDX_MASK);
		vTaskDelay(xDelay);
	}
}

static void task_led1(void *pvParameters) {
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_IDX_MASK, 1, 0, 0);
	const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;
	
	for (;;) {
		pin_toggle(LED1_PIO, LED1_IDX_MASK);
		vTaskDelay(xDelay/100);
		pin_toggle(LED1_PIO, LED1_IDX_MASK);
		vTaskDelay(xDelay/100);
		pin_toggle(LED1_PIO, LED1_IDX_MASK);
		vTaskDelay(xDelay/100);
		pin_toggle(LED1_PIO, LED1_IDX_MASK);
		vTaskDelay(xDelay/100);
		pin_toggle(LED1_PIO, LED1_IDX_MASK);
		vTaskDelay(xDelay/100);
		pin_toggle(LED1_PIO, LED1_IDX_MASK);
		
		vTaskDelay(xDelay);
		
	}
}


/************************************************************************/
/* main                                                                */
/************************************************************************/

int main(void) {
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	if (xTaskCreate(task_led1, "Led1", TASK_LED1_STACK_SIZE, NULL,
			 TASK_LED1_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led1 task\r\n");
	}
	

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
