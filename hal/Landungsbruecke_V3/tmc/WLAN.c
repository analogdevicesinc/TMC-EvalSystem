#include "hal/HAL.h"
#include "hal/RS232.h"
#include "hal/WLAN.h"
#include "hal/RXTX.h"

#define BUFFER_SIZE  1024
#define INTR_PRI     6

static void init();
static void deInit();
static void tx(uint8_t ch);
static uint8_t rx(uint8_t *ch);
static void txN(uint8_t *str, unsigned char number);
static uint8_t rxN(uint8_t *ch, unsigned char number);
static void clearBuffers(void);
static uint32_t bytesAvailable();

static volatile uint8_t rxBuffer[BUFFER_SIZE];
static volatile uint8_t txBuffer[BUFFER_SIZE];

static volatile uint32_t available = 0;

RXTXTypeDef WLAN =
{
	.init            = init,
	.deInit          = deInit,
	.rx              = rx,
	.tx              = tx,
	.rxN             = rxN,
	.txN             = txN,
	.clearBuffers    = clearBuffers,
	.baudRate        = 115200,
	.bytesAvailable  = bytesAvailable
};

static RXTXBufferingTypeDef buffers =
{
	.rx =
	{
		.read    = 0,
		.wrote   = 0,
		.buffer  = rxBuffer
	},

	.tx =
	{
		.read    = 0,
		.wrote   = 0,
		.buffer  = txBuffer
	}
};


void __attribute__ ((interrupt)) USART1_IRQHandler(void);

static void init()
{

	usart_deinit(USART1);

    rcu_periph_clock_enable(RCU_USART1);


	//TxD with pull-up resistor
	 gpio_mode_set(HAL.IOs->pins->WIRELESS_TX.port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HAL.IOs->pins->WIRELESS_TX.bitWeight);
	 gpio_output_options_set(HAL.IOs->pins->WIRELESS_TX.port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HAL.IOs->pins->WIRELESS_TX.bitWeight);


	 //RxD with pull-up resistor
	  gpio_mode_set(HAL.IOs->pins->WIRELESS_RX.port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HAL.IOs->pins->WIRELESS_RX.bitWeight);
	  gpio_output_options_set(HAL.IOs->pins->WIRELESS_RX.port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HAL.IOs->pins->WIRELESS_RX.bitWeight);

	  gpio_af_set(HAL.IOs->pins->WIRELESS_TX.port, GPIO_AF_7, HAL.IOs->pins->WIRELESS_TX.bitWeight);
	  gpio_af_set(HAL.IOs->pins->WIRELESS_RX.port, GPIO_AF_7, HAL.IOs->pins->WIRELESS_RX.bitWeight);

    usart_baudrate_set(USART1, WLAN.baudRate);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);

    nvic_irq_enable(USART1_IRQn, INTR_PRI, 0);

	usart_flag_clear(USART1, USART_FLAG_CTS);
	usart_flag_clear(USART1, USART_FLAG_LBD);
	usart_flag_clear(USART1, USART_FLAG_TBE);
	usart_flag_clear(USART1, USART_FLAG_TC);
	usart_flag_clear(USART1, USART_FLAG_RBNE);
	usart_flag_clear(USART1, USART_FLAG_IDLE);
	usart_flag_clear(USART1, USART_FLAG_ORERR);
	usart_flag_clear(USART1, USART_FLAG_NERR);
	usart_flag_clear(USART1, USART_FLAG_FERR);
	usart_flag_clear(USART1, USART_FLAG_PERR);

    usart_interrupt_enable(USART1, USART_INT_TBE);
    usart_interrupt_enable(USART1, USART_INT_TC);
    usart_interrupt_enable(USART1, USART_INT_RBNE);

    usart_enable(USART1);

}

static void deInit()
{
    usart_disable(USART1);
    nvic_irq_disable(USART1_IRQn);

	usart_flag_clear(USART1, USART_FLAG_CTS);
	usart_flag_clear(USART1, USART_FLAG_LBD);
	usart_flag_clear(USART1, USART_FLAG_TBE);
	usart_flag_clear(USART1, USART_FLAG_TC);
	usart_flag_clear(USART1, USART_FLAG_RBNE);
	usart_flag_clear(USART1, USART_FLAG_IDLE);
	usart_flag_clear(USART1, USART_FLAG_ORERR);
	usart_flag_clear(USART1, USART_FLAG_NERR);
	usart_flag_clear(USART1, USART_FLAG_FERR);
	usart_flag_clear(USART1, USART_FLAG_PERR);

	clearBuffers();
}

void USART1_IRQHandler(void)
{
	if(USART_STAT0(USART1) & USART_STAT0_RBNE)
	{
		buffers.rx.buffer[buffers.rx.wrote] = USART_DATA(USART1);
		buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
		available++;
	}

	if(USART_STAT0(USART1) & USART_STAT0_TBE)
	{
		if(buffers.tx.read != buffers.tx.wrote)
		{
			USART_DATA(USART1)	= buffers.tx.buffer[buffers.tx.read];
			buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;
		}
		else
		{
		    usart_interrupt_disable(USART1, USART_INT_TBE);
		}
	}

	if(USART_STAT0(USART1) & USART_STAT0_TC)
	{
	    usart_interrupt_flag_clear(USART1, USART_INT_FLAG_TC);
	}
}

static void tx(uint8_t ch)
{
	buffers.tx.buffer[buffers.tx.wrote] = ch;
	buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;
    usart_interrupt_enable(USART1, USART_INT_TBE);

}

static uint8_t rx(uint8_t *ch)
{
	if(buffers.rx.read == buffers.rx.wrote)
		return 0;

	*ch = buffers.rx.buffer[buffers.rx.read];
	buffers.rx.read = (buffers.rx.read + 1) % BUFFER_SIZE;
	available--;

	return 1;
}

static void txN(uint8_t *str, unsigned char number)
{
	for(int32_t i = 0; i < number; i++)
		tx(str[i]);
}

static uint8_t rxN(uint8_t *str, unsigned char number)
{
	if(bytesAvailable() < number)
		return 0;

	for(int32_t i = 0; i < number; i++)
		rx(&str[i]);

	return 1;
}

static void clearBuffers(void)
{
	__disable_irq();
	available         = 0;
	buffers.rx.read   = 0;
	buffers.rx.wrote  = 0;

	buffers.tx.read   = 0;
	buffers.tx.wrote  = 0;
	__enable_irq();
}

static uint32_t bytesAvailable()
{
	return available;
}

// todo ADD 3: Implement WLAN Configuration functionality for Startrampe (LH)
uint32_t checkReadyToSend() { return 0; }
void enableWLANCommandMode() {};
uint32_t checkCmdModeEnabled() { return 0; }
uint32_t handleWLANCommand(BufferCommandTypedef cmd, uint32_t value) {UNUSED(cmd); UNUSED(value); return 1; }
uint32_t getCMDReply() { return 0; }
