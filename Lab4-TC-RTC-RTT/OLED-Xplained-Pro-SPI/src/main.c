#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

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

void LED_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(Pio *pio, uint32_t mask);

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void LED_init(int estado) {
	pmc_enable_periph_clk(LED1_OLED_PIO_ID);
	pio_set_output(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK, estado, 0, 0);
};

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK);  
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

int main (void)
{
	board_init();
	sysclk_init();
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	delay_init();

  // Init OLED
	gfx_mono_ssd1306_init();
	gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	gfx_mono_draw_string("mundo", 50,16, &sysfont);
	
	LED_init(1);
	TC_init(TC0, ID_TC1, 1, 4);
	tc_start(TC0, 1);
  
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
