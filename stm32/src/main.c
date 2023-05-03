// Cabeçalho do programa uart.c
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f3/nvic.h>

#define USER_LED_PIN GPIO13

#define PIN_WHEEL_MOTOR_1 GPIO0 //PA0
#define PIN_WHEEL_MOTOR_2 GPIO1 //PA1
#define PIN_WHEEL_MOTOR_SPEED GPIO6 //TIM3 OC1 (PA6)

#define PIN_DIR_MOTOR_1 GPIO4 //PA4
#define PIN_DIR_MOTOR_2 GPIO5 //PA5
#define PIN_DIR_MOTOR_SPEED GPIO7 //TIM3 OC2 (PA7)

//BT SPECS -> USART2: RX(PA3) TX(PA2)
//SERIAL SPECS -> RX(PA10) TX(PA9)
//PWM SPECS -> TIM3 CH1(PA6) CH2(PA7)
#define TIM3_SELECTED_ARR 1000
#define TIM3_MAX_CNT TIM3_SELECTED_ARR
#define DUTY_TO_OCVAL(DUTY) ((DUTY * TIM3_MAX_CNT) / 100)
#define WHEEL_PWM_CHANNEL TIM_OC1
#define DIR_PWM_CHANNEL TIM_OC2

#define START_DUTY 30 //30%

#define COM_START_CMD 247
#define COM_END_CMD 255

enum ZeccarCOMState
{
	ZCOM_STATE_IDLE,
	ZCOM_STATE_RECEIVING
};

enum ZeccarCOMPacket
{
	ZCOM_PKT_START,
	ZCOM_PKT_WHEEL_CMD,
	ZCOM_PKT_WHEEL_SPEED,
	ZCOM_PKT_TURN_CMD,
	ZCOM_PKT_TURN_QUANTITY,
	ZCOM_PKT_END
};

enum ZeccarWheelDirection
{
	ZWDIR_STOP,
	ZWDIR_FRONT,
	ZWDIR_BACK
};

enum ZeccarTurnDirection
{
	ZTDIR_STRAIGHT,
	ZTDIR_RIGHT,
	ZTDIR_LEFT
};

void uart1_setup(uint32_t baudrate);
void uart2_setup();
void gpioc_setup();
void tim_setup();
void run_wheel_motor(uint8_t wheel_direction, uint8_t speed);
void set_wheel_speed(uint16_t speed);
void set_turn_qtt(uint16_t quantity);
void turn_direction_motor(uint8_t turn_direction, uint8_t quantity);
bool validate_com_packet();

static uint8_t WheelMotorSpeed = 30;
static uint8_t TurnMotorQuantity = 30;
static uint8_t SerialBuffer[6];
static uint8_t SerialBurstCounter = 0;
static enum ZeccarCOMState CurrentComState = ZCOM_STATE_IDLE;
static uint8_t CurrentMotorCommand = 0;
static uint8_t CurrentTurnCommand = 0;
static char UsbBuff[50];

static inline void bt_putc(uint8_t ch)
{
	usart_send_blocking(USART2, ch);
}

static inline void usb_putc(uint8_t data)
{
	usart_send_blocking(USART1, data);
}

void usb_puts(uint8_t* data, uint32_t len)
{
	serial_puts(usb_putc, data, len);
}

void bt_puts(uint8_t* data, uint32_t len)
{
	serial_puts(bt_putc, data, len);
}

void serial_puts(void func_putc(uint8_t), uint8_t* data, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++)
	{
		func_putc(*(data + i));
	}
}

// Enviar caracteres lentamente para a  UART
static void communication_task (void *args __attribute__((unused)))
{
	for (;;)
	{
		gpio_toggle(GPIOC, USER_LED_PIN);

		sprintf(UsbBuff, "WS -> %i; WP -> %i; TS -> %i; TP -> %i;\n\r", CurrentMotorCommand, WheelMotorSpeed, CurrentTurnCommand, TurnMotorQuantity);
		usb_puts((uint8_t*) UsbBuff, strlen(UsbBuff));
		// usb_puts((uint8_t*)"TESTE\n\r", strlen("TESTE\n\r"));

		vTaskDelay (pdMS_TO_TICKS(2000));
	}
}

int main (void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	gpioc_setup();
	uart2_setup();
	uart1_setup(9600);
	tim_setup();

	xTaskCreate (communication_task, "communication", 100, NULL, configMAX_PRIORITIES - 1, NULL);
	vTaskStartScheduler ();

	for (;;)
	{
		// gpio_toggle(GPIOC, USER_LED_PIN);

		// char usbBuff[50];
		// sprintf(usbBuff, "WS -> %i; WP -> %i; TS -> %i; TP -> %i;\n\r", CurrentMotorCommand, WheelMotorSpeed, CurrentTurnCommand, TurnMotorQuantity);
		// usb_puts((uint8_t*) usbBuff, strlen(usbBuff));

		// for(int i = 0; i < 50000; i++) {}
	}
	return 0;
}

void uart1_setup(uint32_t baudrate)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	gpio_set_mode(
		GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_USART1_TX);

	usart_set_baudrate(USART1, baudrate);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
}

void uart2_setup()
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART2);

	gpio_set_mode(
		GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_USART2_TX);

	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	usart_enable_rx_interrupt(USART2);

	nvic_enable_irq(NVIC_USART2_IRQ);

	usart_enable(USART2);
}

void gpioc_setup()
{
	rcc_periph_clock_enable(RCC_GPIOC);

	gpio_set_mode(
		GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13);
}

void tim_setup()
{
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM3);

	/* Enable TIM2 interrupt. */
	// nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM3);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM3 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 5kHz
	 */	
	timer_set_prescaler(TIM3, ((rcc_apb1_frequency * 2) / 7200));

	/* Enable preload. */
	timer_enable_preload(TIM3);
	timer_continuous_mode(TIM3);

	/* Count to selected range */
	timer_set_period(TIM3, TIM3_SELECTED_ARR);

	/* -- OC1 and OC2 configuration -- */
	/* Disable outputs. */
	timer_disable_oc_output(TIM3, TIM_OC1);
	timer_disable_oc_output(TIM3, TIM_OC2);

	/* Configure global mode of lines 1 and 2. */
	timer_disable_oc_clear(TIM3, TIM_OC1);
	timer_disable_oc_clear(TIM3, TIM_OC2);

	timer_enable_oc_preload(TIM3, TIM_OC1);
	timer_enable_oc_preload(TIM3, TIM_OC2);
	
	// timer_set_oc_slow_mode(TIM3, TIM_OC1);
	timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);

	/* Configure OC3. */
	timer_set_oc_polarity_high(TIM3, TIM_OC1);
	timer_set_oc_polarity_high(TIM3, TIM_OC2);

	timer_set_oc_idle_state_set(TIM3, TIM_OC1);
	timer_set_oc_idle_state_set(TIM3, TIM_OC2);

	/* Set the capture compare value for OC3. 50% duty */
	timer_set_oc_value(TIM3, TIM_OC1, DUTY_TO_OCVAL(START_DUTY));
	timer_set_oc_value(TIM3, TIM_OC2, DUTY_TO_OCVAL(START_DUTY));

	/* Reenable outputs. */
	timer_enable_oc_output(TIM3, TIM_OC1);
	timer_enable_oc_output(TIM3, TIM_OC2);

	/* ARR reload enable. */
	timer_enable_preload(TIM3);

	/* Enable outputs in the break subsystem. */
	timer_enable_break_main_output(TIM3);

	/* Counter enable. */
	timer_enable_counter(TIM3);

	/* Enable Channel 3 compare interrupt to recalculate compare values --unused */
	// timer_enable_irq(TIM3, TIM_DIER_CC1IE);
}

void set_wheel_speed(uint16_t speed)
{
	timer_set_oc_value(TIM3, WHEEL_PWM_CHANNEL, DUTY_TO_OCVAL(speed));
	WheelMotorSpeed = speed;
}

void set_turn_qtt(uint16_t quantity)
{
	timer_set_oc_value(TIM3, DIR_PWM_CHANNEL, DUTY_TO_OCVAL(quantity));
	TurnMotorQuantity = quantity;
}

void run_wheel_motor(uint8_t wheel_direction, uint8_t speed)
{
	CurrentMotorCommand = wheel_direction;

	switch(wheel_direction)
	{
		case ZWDIR_STOP:
			gpio_clear(GPIOA, PIN_WHEEL_MOTOR_1);
			gpio_clear(GPIOA, PIN_WHEEL_MOTOR_2);
			set_wheel_speed(0);
			break;

		case ZWDIR_FRONT:
			gpio_set(GPIOA, PIN_WHEEL_MOTOR_2);
			gpio_clear(GPIOA, PIN_WHEEL_MOTOR_1);
			set_wheel_speed(speed);
			break;

		case ZWDIR_BACK:
			gpio_set(GPIOA, PIN_WHEEL_MOTOR_1);
			gpio_clear(GPIOA, PIN_WHEEL_MOTOR_2);
			set_wheel_speed(speed);
			break;
	}
}

void turn_direction_motor(uint8_t turn_direction, uint8_t quantity)
{
	CurrentTurnCommand = turn_direction;
	
	switch (turn_direction)
	{
		case ZTDIR_STRAIGHT:
			gpio_clear(GPIOA, PIN_DIR_MOTOR_1);
			gpio_clear(GPIOA, PIN_DIR_MOTOR_2);
			set_wheel_speed(0);
			break;

		case ZTDIR_RIGHT:
			gpio_set(GPIOA, PIN_DIR_MOTOR_1);
			gpio_clear(GPIOA, PIN_DIR_MOTOR_2);
			set_turn_qtt(quantity);
			break;

		case ZTDIR_LEFT:
			gpio_set(GPIOA, PIN_DIR_MOTOR_2);
			gpio_clear(GPIOA, PIN_DIR_MOTOR_1);
			set_turn_qtt(quantity);
			break;
	}
}

bool validate_com_packet()
{
	return (SerialBuffer[ZCOM_PKT_START] == COM_START_CMD) && (SerialBuffer[ZCOM_PKT_END] == COM_END_CMD);
}

void compute_com_packet()
{
	if(!validate_com_packet())
		return;
	
	run_wheel_motor(SerialBuffer[ZCOM_PKT_WHEEL_CMD], SerialBuffer[ZCOM_PKT_WHEEL_SPEED]);
	turn_direction_motor(SerialBuffer[ZCOM_PKT_TURN_CMD], SerialBuffer[ZCOM_PKT_TURN_QUANTITY]);
}

void usart2_isr(void)
{
	uint8_t newByte = usart_recv(USART2);

	switch (CurrentComState)
	{
		case ZCOM_STATE_IDLE:
			if(newByte == COM_START_CMD)
			{
				SerialBuffer[ZCOM_PKT_START] = newByte;
				SerialBurstCounter = 1;
				CurrentComState = ZCOM_STATE_RECEIVING;
			}
			break;

		case ZCOM_STATE_RECEIVING:
			SerialBuffer[SerialBurstCounter++] = newByte;
			if(SerialBurstCounter > ZCOM_PKT_END)
			{
				compute_com_packet();
				CurrentComState = ZCOM_STATE_IDLE;
			}
			break;
	}
}

/* vApplicationStackOverflowHook is called when a stack overflow occurs.
This is usefull in application development, for debugging.  To use this
hook, uncomment it, and set configCHECK_FOR_STACK_OVERFLOW to 1 in
"FreeRTOSConfig.h" header file. */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName )
{
    for( ;; );
}