/**
 * ADC
 * Rafael Corsi @ insper.edu.br
 * Abril 2017
 *
 * Configura o ADC do SAME70 para fazer leitura
 * do sensor de temperatura interno
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "asf.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/** Header printf */
#define STRING_EOL    "\r"
#define STRING_HEADER "-- AFEC Temperature Sensor Example --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)

/************************************************************************/
/* Globals                                                              */
/************************************************************************/


/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;
volatile bool flag_foward = false;
volatile bool flag_neutral = false;
volatile bool flag_back = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;

/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 11

#define AFEC_CHANNEL_POT 5

/************************************************************************/
/* Callbacks: / Handler                                                 */
/************************************************************************/

/**
 * \brief AFEC interrupt callback function.
 */
static void AFEC_Temp_callback(void)
{
	g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT);
	g_is_conversion_done = true;
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure UART console.
 * BaudRate : 115200
 * 8 bits
 * 1 stop bit
 * sem paridade
 */



static void configure_console(void)
{

	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * converte valor lido do ADC para temperatura em graus celsius
 * input : ADC reg value
 * output: Temperature in celsius
 */
static int32_t convert_adc_to_temp(int32_t ADC_value){

  int32_t ul_vol;
  int32_t ul_temp;

  /*
   * converte bits -> tensão (Volts)
   */
	ul_vol = ADC_value * VOLT_REF / (float) MAX_DIGITAL;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  ul_temp = (ul_vol - 720)  * 100 / 233 + 27;
  return(ul_temp);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

static void config_ADC_TEMP(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_5,	AFEC_Temp_callback, 1);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_POT, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_POT, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Seleciona canal e inicializa conversão */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT);
}

/*
void TC1_Handler(void){
	volatile uint32_t ul_dummy; 

	/ ****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	****************************************************************** /
	ul_dummy = tc_get_status(TC0, 1);

	/ * Avoid compiler warning * /
	UNUSED(ul_dummy);
	if(g_is_conversion_done == true) {
		g_is_conversion_done = false;

		//printf("Temp : %d \r\n", convert_adc_to_temp(g_ul_value));
		//afec_start_software_conversion(AFEC0);
		
	}

}*/
void TC2_Handler(void){
	volatile uint32_t ul_dummy; 

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	char str;
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);
	
	if(g_is_conversion_done == true) {
		g_is_conversion_done = false;
	}
	afec_start_software_conversion(AFEC0);
	if(afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT) > 2700){
		flag_foward = true; //não esquecer de dar false nas flags novas no send
		printf("f");
		str = 'f';
	}
	if((afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT) > 1300) && (afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT) < 2700)){
		flag_neutral = true;  //não esquecer de dar false nas flags novas no send
		printf("n");
		str = 'n';
	}
	if(afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT) < 1300){
		flag_back = true;  //não esquecer de dar false nas flags novas no send
		printf("b");
		str = 'b';
	}
	//printf("Teste POT:  %d \n", afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT));
		

}


/************************************************************************/
/* Main                                                                 */
/************************************************************************/
/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{

	/* Initialize the SAM system. */
	sysclk_init();
	ioport_init();
	board_init();
	
	/** Configura timer TC0, canal 1 */
	// um dos tcs está sendo usado pra imprimir e outro para pegara informação
	// do potenciômetro.
	//TC_init(TC0, ID_TC1, 1, 10); 
	TC_init(TC0, ID_TC2, 2, 10); //pega a info do pot ou escreve na string
	// para mudar a velocidade com que isso acontece é só aumentar ou diminuir 
	// o valor do últimom parâmetro da função acima
	
	if(afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT)> 2000){
		printf("f");
	}

	/* inicializa delay */
	delay_init(sysclk_get_cpu_hz());

	/* inicializa console (printf) */
	configure_console();

	/* inicializa e configura adc */
	config_ADC_TEMP();

	/* Output example information. */
	puts(STRING_HEADER);

	/* incializa conversão ADC */
	afec_start_software_conversion(AFEC0);
	while (1) {
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
