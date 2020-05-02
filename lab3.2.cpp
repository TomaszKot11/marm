#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>
#include <stm32_ll_usart.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_system.h>
#include <cstring>
#include <stm32f4xx_ll_gpio.h>
#include "/home/marm/Pulpit/isixsamples/isixrtos/libisix/include/isix/osstats.h"
// #include "/home/marm/Pulpit/isixsamples/isixrtos/libisix/include/isix/osstats.c"
// #include "/home/marm/Pulpit/isixsamples/isixrtos/libisix/kernel/osstats.c"

namespace structures {
	char input_buffer[100];
	unsigned int current_input_idx = 0;
}


void configure_gpiob_6_7_usart1() {
	LL_GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void configure_USER() {
	// configure PD5 pin as input
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);
}

void configure_leds() {
		// configure leds 
		LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_LOW);
		LL_GPIO_SetPinPull(GPIOD,LL_GPIO_PIN_12, LL_GPIO_PULL_NO);
		LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);

		LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_LOW);
		LL_GPIO_SetPinPull(GPIOD,LL_GPIO_PIN_13, LL_GPIO_PULL_NO);
		LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);

		LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_14, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_LOW);
		LL_GPIO_SetPinPull(GPIOD,LL_GPIO_PIN_14, LL_GPIO_PULL_NO);
		LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_14, LL_GPIO_OUTPUT_PUSHPULL);


		LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_LOW);
		LL_GPIO_SetPinPull(GPIOD,LL_GPIO_PIN_15, LL_GPIO_PULL_NO);
		LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_PUSHPULL);
}

void configure_usart1(){
	// structure for USART configuration 
	LL_USART_InitTypeDef USART_InitStructure;

	// enable clock for USART1
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

	// 	Prędkość: 115200 bitów / sekundę
	USART_InitStructure.BaudRate = 115200;
	USART_InitStructure.DataWidth = LL_USART_DATAWIDTH_8B;
	//  8 bitów danych
	USART_InitStructure.StopBits = LL_USART_STOPBITS_1;
	// 	Brak kontroli parzystości
	USART_InitStructure.Parity = LL_USART_PARITY_NONE;
	// 	hardware flow control is not eabled
	USART_InitStructure.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	// 	Transfer direction
	USART_InitStructure.TransferDirection = LL_USART_DIRECTION_TX_RX; 
	// 	oversampling
	USART_InitStructure.OverSampling = LL_USART_OVERSAMPLING_16;      
	// 	pass the initialization structure 
	LL_USART_Init(USART1, &USART_InitStructure); //TODO: check the error?
	// enable the USART 
	LL_USART_Enable(USART1);
}

void configure_uc() {
	configure_gpiob_6_7_usart1();
	configure_usart1();
	configure_leds();
	configure_USER();
}

void toggle_pin(GPIO_TypeDef *GPIOx, uint32_t PinMask) {
	LL_GPIO_TogglePin(GPIOx, PinMask);
}

void write_buffer(char string_to_send[]){
	const auto size_arr = strlen(string_to_send);
	unsigned int sent_idx = 0;

    while(sent_idx < size_arr) {
        while(!LL_USART_IsActiveFlag_TXE(USART1)) {} // check if USART1 Transmit Data Register Empty Flag is set or not
 
        if (sent_idx == (sent_idx - 1))            
			LL_USART_ClearFlag_TC(USART1); // clear the flag
        
        LL_USART_TransmitData8(USART1, string_to_send[sent_idx++]);
    }
 
    while(!LL_USART_IsActiveFlag_TC(USART1)){}
}

uint8_t receive_data()
{
    if(LL_USART_IsActiveFlag_RXNE(USART1)){ // check the Not Empty Read Data Register Flag
        return LL_USART_ReceiveData8(USART1);
    } 

    return (uint8_t)1;
}

void handle_input(char c) {
	if(c != 1)
		structures::input_buffer[structures::current_input_idx++] = c;

	if(strcmp("led -n 2", structures::input_buffer) == 0) {
		toggle_pin(GPIOD, LL_GPIO_PIN_12);
		structures::current_input_idx = 0;
		memset(structures::input_buffer, 0, 100);
	} else if(strcmp("led -n 3", structures::input_buffer) == 0) {
		toggle_pin(GPIOD, LL_GPIO_PIN_13);
		structures::current_input_idx = 0;
		memset(structures::input_buffer, 0, 100);
	} else if(strcmp("led -n 4", structures::input_buffer) == 0) {
		toggle_pin(GPIOD, LL_GPIO_PIN_14);
		structures::current_input_idx = 0;
		memset(structures::input_buffer, 0, 100);
	} else if(strcmp("led -n 5", structures::input_buffer) == 0) {
		toggle_pin(GPIOD, LL_GPIO_PIN_15);
		structures::current_input_idx = 0;
		memset(structures::input_buffer, 0, 100);
	} else if(strcmp("user", structures::input_buffer)==0) {
		const auto is_pressed = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
		char buffer[100];

		structures::current_input_idx = 0;
		memset(structures::input_buffer, 0, 100);

		if(is_pressed) {
			const auto is_pressed_two = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
			if(is_pressed_two) {
				sprintf(buffer, "USER button clicked");
				write_buffer(buffer);
				return;
			}
		}

		sprintf(buffer, "USER button not clicked");
		write_buffer(buffer);
	} else if(strcmp("heap", structures::input_buffer) == 0) {
			char buffer[100];

			structures::current_input_idx = 0;
			memset(structures::input_buffer, 0, 100);

			isix_memory_stat_t mem_info;
			isix::heap_stats(mem_info);
			sprintf(buffer, "\nIlosc wolnej pamieci to: %d\n\r", mem_info.free);
			write_buffer(buffer);
			return;
	} else if(strcmp("cpu", structures::input_buffer) == 0) {
		char buffer[100];

		int cpu_usage = isix::cpuload();
		
		sprintf(buffer, "\nAktuale zuzycie procesora to: %f \n\r", (cpu_usage / 10.0));
		write_buffer(buffer);
		return;
	} else if(strcmp("clean", structures::input_buffer) == 0) {
		structures::current_input_idx = 0;
		memset(structures::input_buffer, 0, 100);
	}
}

namespace app {
    auto lab3_thread() -> void
    {
		char buffer[100];
		unsigned int i;
	
		configure_uc();
		// program loop
		for(i = 0;;++i){
            char c = receive_data();
            if (c != 1) {
				handle_input(c);
				sprintf(buffer, "Kliniete: %c\n\r", c);
                write_buffer(buffer);
            }
        }
	}
}


auto main() -> int
{
	static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );
	dblog_init_locked(
		[](int ch, void*) {
			return periph::drivers::uart_early::putc(ch);
		},
		nullptr,
		[]() {
			m_ulock_sem.wait(ISIX_TIME_INFINITE);
		},
		[]() {
			m_ulock_sem.signal();
		},
		periph::drivers::uart_early::open, "serial0", 115200
	);
    static constexpr auto stack_size = 2048;
    static auto thread1 = 
        isix::thread_create_and_run(stack_size,isix::get_min_priority(),0,app::lab3_thread);
    dbprintf("Lab3 - Serial port");
	isix::start_scheduler();
	return 0;
}
