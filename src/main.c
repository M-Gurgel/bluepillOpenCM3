#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/systick.h>


// interrupt variable 
int irq_up = 0;

// RTOS stackovreflow prevention
void vApplicationStackOverflowHook(
	TaskHandle_t xTask __attribute__((unused)),
    char *pcTaskName __attribute__((unused))) {

	while(1);
}

static void ledsTask(void *args __attribute__((unused))) {
	
	int i = 0;
	//int g_blink_mode;
	while(1) {
		i++;
		//g_blink_mode = gpio_get(GPIOA, GPIO7);
		if(gpio_get(GPIOA, GPIO7)) {
			//200 ms
			gpio_toggle(GPIOA, GPIO1);
			
			//400 ms
			if(i%2 == 0)
				gpio_toggle(GPIOA, GPIO2);

			//600 ms
			if(i%3 == 0)
				gpio_toggle(GPIOA, GPIO3);

			//800 ms
			if(i%4 == 0)
				gpio_toggle(GPIOA, GPIO4);

		}
		else {
			if(i%2) {
				gpio_set(GPIOA, GPIO1 | GPIO3);
				gpio_clear(GPIOA, GPIO2 | GPIO4);
			}
			else{
				gpio_set(GPIOA, GPIO2 | GPIO4);
				gpio_clear(GPIOA, GPIO1 | GPIO3);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(200));
	}
}


static void countupTask(void *args __attribute__((unused))) {
	int min = 0;
	int seg = 0;
	int cent = 0;

	while(1) {
		//printf("Cronometro: %.2d:%.2d:%.2d\n", min, seg, cent);
		cent++;

		if(cent >= 100) {
			cent = 0;
			seg++;
		}

		if(seg >= 60) {
			cent = 0;
			seg = 0;
			min++;
		}

		// Limite artificial 10 min.
		if(min >= 10)
		{
			cent = 0;
			seg = 0;
			min = 0;
		}

		// 10ms = 1 cent. seg
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}


// pwm Task
static void pwmTask(void *args __attribute__((unused))) {
	
	//0 - 1000 value
	int step = 0;
	int going_up = 1;

	while(1) {
		
		if(going_up) {
			step+= 5;
			TIM_CCR1(TIM1) = step;
		}
		else
		{
			step -= 5;
			TIM_CCR1(TIM1) = step;
		}

		if(step >= 1000) going_up = 0;
		if(step <= 0) going_up = 1;	

		//5 ms for 200 steps in one second
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	

}

// Interrupt routine
void exti0_isr(void)
{
	exti_reset_request(EXTI0);

	if (irq_up == 0) {
		gpio_set(GPIOA, GPIO7);
		irq_up = 1;
		exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	} else {
		gpio_clear(GPIOA, GPIO7);
		irq_up = 0;
		exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	}
}

// printf "emulation"
// Used by printf to set the default outport
int _write(int file, char *ptr, int len)
{
  int i;
  
  if (file == 1) {
    for (i = 0; i < len; i++) {
      usart_send_blocking(USART3, ptr[i]);
    }
    return i;
  }
  
  return -1;
}

// Program Start
int main(void) {
    // Setup main clock
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Enable clock for GPIO channels
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	// Enable clock for USART
	rcc_periph_clock_enable(RCC_USART3);

	// Set LEDs GPIO
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO7);

  	// setup usart3
  	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART3, USART_MODE_TX);
	usart_enable(USART3);

	// Setup Button Interrrupt
	rcc_periph_clock_enable(RCC_AFIO);
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);
	exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI0);

	// Setup PWM
	rcc_periph_clock_enable(RCC_TIM1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8);
	//timer_reset(TIM1);
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM1, 72-1); // Using Clock at 72Mhz
    timer_set_period(TIM1, 1000-1); //
	timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
	timer_enable_oc_preload(TIM1, TIM_OC1);
	timer_enable_break_main_output(TIM1);
	timer_enable_oc_output(TIM1, TIM_OC1);
	timer_enable_counter(TIM1);
	TIM_CCR1(TIM1) = 0; // Initial Value

	xTaskCreate(ledsTask, "Blink", 100, NULL, 2, NULL);

	xTaskCreate(countupTask, "UpCounter", 100, NULL, 2, NULL);

	xTaskCreate(pwmTask, "PWMCControl", 100, NULL, 2, NULL);

    // Start RTOS
	vTaskStartScheduler();

	return 0;
}

