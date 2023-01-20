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

static void init()
{

}

static void deInit()
{

}

static void tx(uint8_t ch)
{

}

static uint8_t rx(uint8_t *ch)
{
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
