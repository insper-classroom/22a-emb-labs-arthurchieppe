#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

//Pino X
#define X_PIO					PIOD
#define X_PIO_ID				ID_PIOD
#define X_PIO_IDX				27
#define X_PIO_IDX_MASK			(1u << X_PIO_IDX)

//Pino Y
#define Y_PIO					PIOA
#define Y_PIO_ID				ID_PIOA
#define Y_PIO_IDX				3
#define Y_PIO_IDX_MASK			(1u << Y_PIO_IDX)

// Configuracoes do botao da placa OLED:
#define BUT1_PIO				PIOD
#define BUT1_PIO_ID				ID_PIOD
#define BUT1_PIO_IDX			28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX) // esse já está pronto.

// LED1 placa oled
#define LED1_OLED_PIO		PIOA
#define LED1_OLED_PIO_ID	ID_PIOA
#define LED1_OLED_PIO_IDX	0
#define LED1_OLED_PIO_IDX_MASK  (1 << LED1_OLED_PIO_IDX)

// LED2 placa oled
#define LED2_OLED_PIO		PIOC
#define LED2_OLED_PIO_ID	ID_PIOC
#define LED2_OLED_PIO_IDX	30
#define LED2_OLED_PIO_IDX_MASK  (1 << LED2_OLED_PIO_IDX)

// LED3 placa oled
#define LED3_OLED_PIO		PIOB
#define LED3_OLED_PIO_ID	ID_PIOB
#define LED3_OLED_PIO_IDX	2
#define LED3_OLED_PIO_IDX_MASK  (1 << LED3_OLED_PIO_IDX)

//Global chars:
volatile char echo_fall_flag = 1;
volatile char echo_rise_flag = 0;
volatile char but1_flag = 0;

//RTT
void io_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);


void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void pisca_led (int n, int t) {
	for (int i=0;i<n;i++){
		pio_clear(LED3_OLED_PIO, LED3_OLED_PIO_IDX_MASK);
		delay_ms(t);
		pio_set(LED3_OLED_PIO, LED3_OLED_PIO_IDX_MASK);
		delay_ms(t);
	}
}

void LED_init(int estado) {
	pmc_enable_periph_clk(LED1_OLED_PIO_ID);
	pio_set_output(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK, estado, 0, 0);
	pmc_enable_periph_clk(LED2_OLED_PIO_ID);
	pio_set_output(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK, estado, 0, 0);
	pmc_enable_periph_clk(LED3_OLED_PIO_ID);
	pio_set_output(LED3_OLED_PIO, LED3_OLED_PIO_IDX_MASK, estado, 0, 0);
}


void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK);
		RTT_init(1, 2, RTT_MR_RTTINCIEN);
	}
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		;    // BLINK Led
	}

}


static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}


void but1_callback(void) {
	but1_flag = 1;
}

void io_init(void) {
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but1_callback);
	
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 3);
}	


int main (void)
{
	board_init();
	sysclk_init();
	io_init();
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	delay_init();

  // Init OLED
	gfx_mono_ssd1306_init();
	
	LED_init(1);
	
	//RTT
	RTT_init(1, 2, RTT_MR_ALMIEN); 
	
	char str[128];
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		if (but1_flag) {
			gfx_mono_draw_string("           ", 0, 0, &sysfont);
			sprintf(str, "ED %d", 1);
			gfx_mono_draw_string(str, 0,0, &sysfont);
			
		}
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
