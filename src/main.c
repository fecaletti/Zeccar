// Cabeçalho do programa uart.c
#include <FreeRTOS.h>
#include <task.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

//PC13 --> USER LED
#define SKIP_UART

// Configuração do periférico UART
static void uart_setup (void)
{
	rcc_periph_clock_enable (RCC_GPIOA);
	rcc_periph_clock_enable (RCC_USART1);

	// UART TX pino PA9 (GPIO_USART1_TX)
	gpio_set_mode (
		GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_USART1_TX);

	usart_set_baudrate (USART1, 115200);
	usart_set_databits (USART1, 8);
	usart_set_stopbits (USART1,USART_STOPBITS_1);
	usart_set_mode (USART1,USART_MODE_TX);
	usart_set_parity (USART1,USART_PARITY_NONE);
	usart_set_flow_control (USART1,USART_FLOWCONTROL_NONE);
	usart_enable (USART1);
}

// Enviar um caracter para a UART
static inline void uart_putc (char ch)
{
	usart_send_blocking (USART1,ch);
}

// Enviar caracteres lentamente para a  UART
static void task1 (void *args __attribute__((unused)))
{
	int c = '0' - 1;

	for (;;)
	{
		// Pisca o LED PC13
		gpio_toggle (GPIOC,GPIO13);
		// Tarefa controlada pelo agendador do FreeRTOS
		vTaskDelay (pdMS_TO_TICKS(100));
		// Controle de fluxo do programa
		#ifdef SKIP_UART
		continue;
		#endif
		if ( ++c >= 'Z' )
		// Envia o último carácter e os caracteres de controle pela USART
		{
			uart_putc (c);
			uart_putc ('\r');
			uart_putc ('\n');
			c = '0' - 1;
		} 
        else
		// Envia carácter pela USART
		{
			uart_putc (c);
		}
	}
}

// Programa Main
int main (void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz ();

	// Configura LED PC13
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode (
		GPIOC,
                	GPIO_MODE_OUTPUT_2_MHZ,
                	GPIO_CNF_OUTPUT_PUSHPULL,
                	GPIO13);

	// Configura o periférico USART1
	// uart_setup ();
	// Criar a única tarefa para o FreeRTOS
	xTaskCreate (task1,"task1",100,NULL,configMAX_PRIORITIES-1,NULL);
	// Ativa o agendador de tarefa do FreeRTOS
	vTaskStartScheduler ();

	for (;;);
	return 0;
}

/* vApplicationStackOverflowHook is called when a stack overflow occurs.
This is usefull in application development, for debugging.  To use this
hook, uncomment it, and set configCHECK_FOR_STACK_OVERFLOW to 1 in
"FreeRTOSConfig.h" header file. */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName )
{
    for( ;; );
}