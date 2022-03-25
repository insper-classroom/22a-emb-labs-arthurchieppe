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
#define ECHO_PIO					PIOA
#define ECHO_PIO_ID				ID_PIOA
#define ECHO_PIO_IDX				3
#define ECHO_PIO_IDX_MASK			(1u << ECHO_PIO_IDX)

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

#define MIN_TEMPO			58*1e-6
#define MAX_TEMPO			11764*1e-6
#define SOUND				340

//Global chars:
volatile char echo_fall_flag = 0;
volatile char echo_rise_flag = 0;
volatile char but1_flag = 0;
volatile char timeout_counter = 0;

//RTT
void io_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

//TC
volatile char start_measure = 0;

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	* Isso ? realizado pela leitura do status do perif?rico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	//pin_toggle(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK);
	start_measure = 1;
	timeout_counter++;
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void RTT_Handler(void) {
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
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

void echo_callback(void) {
	if (!pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK)) {
		echo_fall_flag = 1;
		echo_rise_flag = 0;
		} else if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK)) {
		echo_fall_flag = 0;
		echo_rise_flag = 1;
	}
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
	
	//X (Trig):
	pmc_enable_periph_clk(X_PIO_ID);
	pio_configure(X_PIO, PIO_OUTPUT_0, X_PIO_IDX_MASK, PIO_DEFAULT);
	
	//Y (Echo):
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_PIO_IDX_MASK, PIO_DEFAULT);
	//pio_set_debounce_filter(ECHO_PIO, ECHO_PIO_IDX_MASK, 60);
	pio_handler_set(ECHO_PIO,ECHO_PIO_ID,ECHO_PIO_IDX_MASK,PIO_IT_EDGE, echo_callback);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_IDX_MASK);
	pio_get_interrupt_status(ECHO_PIO);
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4); // Prioridade 4
}	

void trig_x(void) {
	pio_set(X_PIO, X_PIO_IDX_MASK);
	delay_us(10);
	pio_clear(X_PIO, X_PIO_IDX_MASK);
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
	
	//RTT
	//RTT_init(1, 2, RTT_MR_ALMIEN); 
	TC_init(TC0, ID_TC1, 1, 2);
	tc_start(TC0, 1);
	
	char str[128];
	int rtt_counter = 0;

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		if (start_measure) {
			trig_x();
			rtt_counter = 0;
			start_measure = 0;
		}
		if (timeout_counter > 2) {
			gfx_mono_draw_string("             ", 0, 0, &sysfont);
			gfx_mono_draw_string("CHECK SENSOR", 0,0, &sysfont);
		}
		
		if (echo_rise_flag && !start_measure) {
			timeout_counter = 0;
			RTT_init(1.0/(2*MIN_TEMPO), 0, 0);
			echo_rise_flag = 0;
		}
		if (echo_fall_flag && !start_measure) {
			rtt_counter = rtt_read_timer_value(RTT);
			double seconds = rtt_counter*MIN_TEMPO;
			double distance = SOUND*seconds*100;
			if (distance > 400 ) {
				gfx_mono_draw_string("             ", 0, 0, &sysfont);
				gfx_mono_draw_string("TOO FAR", 0,0, &sysfont);
			} else {
				gfx_mono_draw_string("           ", 0, 0, &sysfont);
				sprintf(str, " %0.2lf cm", distance);
				gfx_mono_draw_string(str, 0,0, &sysfont);
			}
			
			but1_flag = 0;
			echo_fall_flag = 0;
			start_measure = 1;
			timeout_counter = 0;
		}
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
