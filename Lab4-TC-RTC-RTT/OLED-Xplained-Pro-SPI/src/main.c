#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// Configuracoes do botao da placa OLED:
#define BUT1_PIO				PIOD
#define BUT1_PIO_ID				ID_PIOD
#define BUT1_PIO_IDX			28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX) // esse j? est? pronto.



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

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;
	
//TC
void LED_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(Pio *pio, uint32_t mask);
volatile char counter = 0;

//RTT
void io_init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

//RTC
volatile char flag_rtc_alarm = 0;
volatile char but1_flag = 0;
volatile char clock_flag = 0;
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void pisca_led(int n, int t);

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

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	* Isso ? realizado pela leitura do status do perif?rico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK);  
}

void TC3_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	* Isso ? realizado pela leitura do status do perif?rico
	**/
	volatile uint32_t status = tc_get_status(TC1, 0);
	if (but1_flag) {
		counter += 1;
	}
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
		pin_toggle(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK);
		RTT_init(1, 2, RTT_MR_RTTINCIEN);
	}
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		;    // BLINK Led
	}

}

void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);	
	
	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		clock_flag = 1;
		// o c?digo para irq de segundo vem aqui
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		// o c?digo para irq de alame vem aqui
		//flag_rtc_alarm = 1;
	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
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

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
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
	//TC
	TC_init(TC0, ID_TC1, 1, 4);
	TC_init(TC1, ID_TC3, 0, 1);
	
	tc_start(TC0, 1);
	tc_start(TC1, 0);
	//RTT
	RTT_init(1, 2, RTT_MR_ALMIEN); 
	
	/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	
	/* Leitura do valor atual do RTC */
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	char str[128];
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		if (clock_flag) {
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			gfx_mono_draw_string("           ", 0, 0, &sysfont);
			if (current_sec > 9) {
				sprintf(str, "%d:%d:%d", current_hour, current_min, current_sec);
			} else {
				sprintf(str, "%d:%d:0%d", current_hour, current_min, current_sec);
			}
			
			gfx_mono_draw_string(str, 0,0, &sysfont);
			clock_flag = 0;
			
		}
		//if (but1_flag) {
			//rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			//rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
			//rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);
			//rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 20);
			//but1_flag = 0;
		//}
		//if (flag_rtc_alarm) {
			//pisca_led(1,500);
			//flag_rtc_alarm = 0;
		//}
		if (counter >= 20) {
			pisca_led(1, 500);
			counter = 0;
			but1_flag = 0;
		}
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
