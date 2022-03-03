/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

//#define LED_PIO           PIOC
//#define LED_PIO_ID        ID_PIOC
//#define LED_PIO_IDX       8
//#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)
//
//// Configuracoes do botao da placa:
//#define BUT_PIO				PIOA
//#define BUT_PIO_ID			ID_PIOA
//#define BUT_PIO_IDX			11
//#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX) // esse já está pronto.

// Configuracoes do botao da placa OLED:
#define BUT1_PIO				PIOD
#define BUT1_PIO_ID				ID_PIOD
#define BUT1_PIO_IDX			28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX) // esse já está pronto.

// Configuracoes do botao da placa OLED:
#define BUT2_PIO				PIOC
#define BUT2_PIO_ID				ID_PIOC
#define BUT2_PIO_IDX			31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX) // esse já está pronto.

// Configuracoes do botao da placa OLED:
#define BUT3_PIO				PIOA
#define BUT3_PIO_ID				ID_PIOA
#define BUT3_PIO_IDX			19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX) // esse já está pronto.




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

//LAB 2:
/**
 * \brief Set a high output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}

/**
 * \brief Set a low output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}

/**
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_pull_up_enable Indicates if the pin(s) internal pull-up shall be
 * configured.
 */
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask,
        const uint32_t ul_pull_up_enable){
	p_pio->PIO_PUER = ul_mask;
 }

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	pmc_enable_periph_clk(LED1_OLED_PIO_ID);
	pmc_enable_periph_clk(LED2_OLED_PIO_ID); 
	pmc_enable_periph_clk(LED3_OLED_PIO_ID);  
	// Inicializa PIO do botao
	//pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

	
	//Inicializa PC8 como saída
	pio_set_output(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED3_OLED_PIO, LED3_OLED_PIO_IDX_MASK, 0, 0, 0);
	// configura pino ligado ao botão como entrada com um pull-up.
	//pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, PIO_DEFAULT);
	// Inicializa botao como energizado:
	//_pio_pull_up(BUT_PIO, BUT_PIO_IDX_MASK, 1);
	_pio_pull_up(BUT1_PIO, BUT1_PIO_IDX_MASK, 1);
	_pio_pull_up(BUT2_PIO, BUT2_PIO_IDX_MASK, 1);
	_pio_pull_up(BUT3_PIO, BUT3_PIO_IDX_MASK, 1);


}



/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {
	if (!pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		for (int i =0; i < 5; i++) {
			
			_pio_set(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK);      // Coloca 1 no pino LED
			delay_ms(200);                        // Delay por software de 200 ms
			_pio_clear(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
			delay_ms(200);                        // Delay por software de 200 ms
			}
		
		} else if (!pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)) {
			for (int i =0; i < 5; i++) {
				
				_pio_set(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK);      // Coloca 1 no pino LED
				delay_ms(200);                        // Delay por software de 200 ms
				_pio_clear(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
				delay_ms(200);                        // Delay por software de 200 ms
			}
			
		} else if (!pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)) {
			for (int i =0; i < 5; i++) {
				
				_pio_set(LED3_OLED_PIO, LED3_OLED_PIO_IDX_MASK);      // Coloca 1 no pino LED
				delay_ms(200);                        // Delay por software de 200 ms
				_pio_clear(LED3_OLED_PIO, LED3_OLED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
				delay_ms(200);                        // Delay por software de 200 ms
			}
		} else {
			_pio_set(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK);
			_pio_set(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK);
			_pio_set(LED3_OLED_PIO, LED3_OLED_PIO_IDX_MASK);
		}
	/*} else if (!pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)) {
		for (int i =0; i < 10; i++) {
			_pio_set(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK);      // Coloca 1 no pino LED
			delay_ms(200);                        // Delay por software de 200 ms
			_pio_clear(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
			delay_ms(200);                        // Delay por software de 200 ms
		}
		
	  
	_pio_set(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK);
	_pio_set(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK); 
	_pio_set(LED3_OLED_PIO, LED3_OLED_PIO_IDX_MASK);       // Coloca 1 no pino LED
	delay_ms(200);                        // Delay por software de 200 ms
	_pio_clear(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK);
	_pio_clear(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK); 
	_pio_clear(LED3_OLED_PIO, LED3_OLED_PIO_IDX_MASK);     // Coloca 0 no pino do LED
	delay_ms(200);                        // Delay por software de 200 ms
		
		
	*/	
  }
  return 0;
}

//Codigo anterior entrega obrigatoria:
/*
if (!pio_get(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK)) {
	for (int i =0; i < 10; i++) {
		
		_pio_set(LED_PIO, LED_PIO_IDX_MASK);      // Coloca 1 no pino LED
		delay_ms(200);                        // Delay por software de 200 ms
		_pio_clear(LED_PIO, LED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
		delay_ms(200);                        // Delay por software de 200 ms
	}
	} else {
	_pio_set(LED_PIO, LED_PIO_IDX_MASK);
}
*/
