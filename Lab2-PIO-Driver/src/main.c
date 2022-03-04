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



/************************************************************************/
/* constants                                                            */
/************************************************************************/
/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)

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
	if (ul_pull_up_enable == 1) {
		p_pio->PIO_PUER = ul_mask;
	} else {
		p_pio->PIO_PUDR = ul_mask;
	}
	
 }
 
/**
 * \brief Configure one or more pin(s) or a PIO controller as inputs.
 * Optionally, the corresponding internal pull-up(s) and glitch filter(s) can
 * be enabled.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure as input(s).
 * \param ul_attribute PIO attribute(s).
 */
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute)
{
	_pio_pull_up(p_pio, ul_mask, ul_attribute & PIO_PULLUP);
	if (ul_attribute & PIO_DEBOUNCE) {
		p_pio->PIO_IFSCDR = ul_mask;
		} else {
		if (ul_attribute & PIO_DEGLITCH) {
			p_pio->PIO_IFSCER = ul_mask;
		}
	}
	if (ul_attribute && (PIO_DEGLITCH || PIO_DEBOUNCE)) {
		p_pio->PIO_IFER = ul_mask;
		} else {
		p_pio->PIO_IFDR = ul_mask;
	}
}

/**
 * \brief Configure one or more pin(s) of a PIO controller as outputs, with
 * the given default value. Optionally, the multi-drive feature can be enabled
 * on the pin(s).
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure.
 * \param ul_default_level Default level on the pin(s).
 * \param ul_multidrive_enable Indicates if the pin(s) shall be configured as
 * open-drain.
 * \param ul_pull_up_enable Indicates if the pin shall have its pull-up
 * activated.
 */
void _pio_set_output(Pio *p_pio, const uint32_t ul_mask,
        const uint32_t ul_default_level,
        const uint32_t ul_multidrive_enable,
        const uint32_t ul_pull_up_enable)
{
	pio_pull_up(p_pio, ul_mask, ul_pull_up_enable);
	if (ul_default_level) {
		_pio_set(p_pio, ul_mask);
	} else {
		_pio_clear(p_pio, ul_mask);
	}
	
	if (ul_multidrive_enable) {
		p_pio->PIO_MDER = ul_mask;
	} else {
		p_pio->PIO_MDDR = ul_mask;
	}
	p_pio->PIO_OER = ul_mask;
	p_pio->PIO_PER = ul_mask;
}

/**
 * \brief Return 1 if one or more PIOs of the given Pin instance currently have
 * a high level; otherwise returns 0. This method returns the actual value that
 * is being read on the pin. To return the supposed output value of a pin, use
 * pio_get_output_data_status() instead.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_type PIO type.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 *
 * \retval 1 at least one PIO currently has a high level.
 * \retval 0 all PIOs have a low level.
 */
uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type,
        const uint32_t ul_mask)
{
	uint32_t ul_reg;

	if (ul_type == PIO_OUTPUT_0) {
		ul_reg = p_pio->PIO_ODSR;
		} 
	else if (ul_type == PIO_INPUT) {
		ul_reg = p_pio->PIO_PDSR;
	}

	if ((ul_reg & ul_mask) == 0) {
		return 0;
		} else {
		return 1;
	}
}



 
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
// 	pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, PIO_DEFAULT);
// 	pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, PIO_DEFAULT);
// 	pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, PIO_DEFAULT);
// 	// Inicializa botao como energizado:
// 	//_pio_pull_up(BUT_PIO, BUT_PIO_IDX_MASK, 1);
// 	_pio_pull_up(BUT1_PIO, BUT1_PIO_IDX_MASK, 1);
// 	_pio_pull_up(BUT2_PIO, BUT2_PIO_IDX_MASK, 1);
// 	_pio_pull_up(BUT3_PIO, BUT3_PIO_IDX_MASK, 1);
	_pio_set_input(BUT1_PIO, BUT1_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT2_PIO, BUT2_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BUT3_PIO, BUT3_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);





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
	if (!_pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		for (int i =0; i < 5; i++) {
			
			_pio_set(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK);      // Coloca 1 no pino LED
			delay_ms(200);                        // Delay por software de 200 ms
			_pio_clear(LED1_OLED_PIO, LED1_OLED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
			delay_ms(200);                        // Delay por software de 200 ms
			}
		
		} else if (!_pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)) {
			for (int i =0; i < 5; i++) {
				
				_pio_set(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK);      // Coloca 1 no pino LED
				delay_ms(200);                        // Delay por software de 200 ms
				_pio_clear(LED2_OLED_PIO, LED2_OLED_PIO_IDX_MASK);    // Coloca 0 no pino do LED
				delay_ms(200);                        // Delay por software de 200 ms
			}
			
		} else if (!_pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)) {
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
  }
  return 0;
}

