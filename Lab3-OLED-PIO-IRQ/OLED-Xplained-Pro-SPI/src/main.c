#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

//Defines PIO
// LED
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Bot�o 0 placa
#define BUT0_PIO      PIOA
#define BUT0_PIO_ID   ID_PIOA
#define BUT0_IDX  11
#define BUT0_IDX_MASK (1 << BUT0_IDX)

// Configuracoes do botao da placa OLED:
#define BUT1_PIO				PIOD
#define BUT1_PIO_ID				ID_PIOD
#define BUT1_PIO_IDX			28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX) // esse j� est� pronto.

//Globals:
volatile char but0_flag;
volatile char but1_flag;
volatile char but1_fall_flag;
volatile char but1_rise_flag;
int delay = 200;

/* prototype                                                            */
/************************************************************************/
void io_init(void);
void pisca_led(int n);

/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/
void but1_callback(void) {
	if (!pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		but1_fall_flag = 1;
		but1_rise_flag = 0;
	} else if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		but1_fall_flag = 0;
		but1_rise_flag = 1;
	}
}

void but0_callback(void) {
	but0_flag = 1;
}



/************************************************************************/
/* fun��es                                                              */
/************************************************************************/
// pisca led N vez no periodo T
void pisca_led(int n){
	for (int i=0;i<n;i++){
		//if (but1_fall_flag) {
			//break;
		//}
		pio_set(LED_PIO, LED_IDX_MASK);
		delay_ms(delay);
		pio_clear(LED_PIO, LED_IDX_MASK);
		delay_ms(delay);
	}
}

void update_display() {
	char str[128];
	sprintf(str, "%d ms", delay);
	gfx_mono_draw_string(str, 50,16, &sysfont);
}

// Inicializa botao SW0 do kit com interrupcao
void io_init(void)
{

	// Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do perif�rico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT0_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);

	// Configura PIO para lidar com o pino do bot�o como entrada
	// com pull-up
	//Botao 0:
	pio_configure(BUT0_PIO, PIO_INPUT, BUT0_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT0_PIO, BUT0_IDX_MASK, 60);
	//Botao 1:
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);

	// Configura interrup��o no pino referente ao botao e associa
	// fun��o de callback caso uma interrup��o for gerada
	// a fun��o de callback � a: but_callback()
	pio_handler_set(BUT0_PIO,
	BUT0_PIO_ID,
	BUT0_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but0_callback);
	
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX_MASK,
	PIO_IT_EDGE,
	but1_callback);

	// Ativa interrup��o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT0_PIO, BUT0_IDX_MASK);
	pio_get_interrupt_status(BUT0_PIO);
	
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr�ximo de 0 maior)
	NVIC_EnableIRQ(BUT0_PIO_ID);
	NVIC_SetPriority(BUT0_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
}


int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	io_init();
	
	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Init OLED
	gfx_mono_ssd1306_init();
  
  
	gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	char str[128];
	sprintf(str, "%d ms", delay);
	gfx_mono_draw_string(str, 50,16, &sysfont);
  
  

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		while (but1_fall_flag) {
			int counter = 0;
			while (but1_fall_flag) {
				counter++;
				if (counter > 300000000) {
					delay -= 100;
					if (delay <= 0) {
						delay = 100;
					}
					update_display();
					delay_ms(100);
				}
			}
			if (counter <= 300000000) {
				delay += 100;
				update_display();
				delay_ms(100);
			}
		}
		if (but0_flag) {
			pisca_led(10); //Mudar para 30
			update_display();
			but0_flag = 0;	
			}
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	

			//// Escreve na tela um circulo e um texto BARRA DE PROGRESSO
			//
			//for(int i=70;i<=120;i+=2){
				//
				//gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_SET);
				//delay_ms(10);
				//
			//}
			//
			//for(int i=120;i>=70;i-=2){
				//
				//gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_CLR);
				//delay_ms(10);
				//
			//}
			
			
	}
}
