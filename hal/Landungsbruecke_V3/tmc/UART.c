/*******************************************************************************
* Copyright © 2023 Analog Devices, Inc.
*******************************************************************************/

#include <string.h> // for memcpy
#include "hal/HAL.h"
#include "hal/UART.h"

#define INTR_PRI     6
#define UART_DEFAULT_TIMEOUT_VALUE 10 // [ms]

// DMA0 assignments for the different UARTs
// Note: This implementation assumes all channels use DMA0!
// DMA0 Channel 1, Subperipheral 4: USART2_RX
#define UART2_DMA_RX_CHANNEL    DMA_CH1
#define UART2_DMA_RX_SUBPERIPH  DMA_SUBPERI4
// DMA0 Channel 3, Subperipheral 4: USART2_TX
#define UART2_DMA_TX_CHANNEL    DMA_CH3
#define UART2_DMA_TX_SUBPERIPH  DMA_SUBPERI4

// DMA0 Channel 2, Subperipheral 4: USART3_RX
#define UART3_DMA_RX_CHANNEL    DMA_CH2
#define UART3_DMA_RX_SUBPERIPH  DMA_SUBPERI4
// DMA0 Channel 4, Subperipheral 4: USART3_TX
#define UART3_DMA_TX_CHANNEL    DMA_CH4
#define UART3_DMA_TX_SUBPERIPH  DMA_SUBPERI4

// RX buffer: Maximum size dictates how much data can be held before losing bytes
#define UART_RX_DMA_BUFFER_SIZE 32
// TX buffer: Maximum size dictates how much data can be pushed to be transferred
// without having to block
#define UART_TX_DMA_BUFFER_SIZE 16

static uint8_t uartTXDMABuffer[UART_TX_DMA_BUFFER_SIZE];
static uint8_t uartRXDMABuffer[UART_RX_DMA_BUFFER_SIZE];
static uint32_t uartRXReadIndex = 0;
static uint32_t uartRXEchoBytes = 0;



static void init();
static void deInit();
static void tx(uint8_t ch);
static uint8_t rx(uint8_t *ch);
static void txN(uint8_t *str, unsigned char number);
static uint8_t rxN(uint8_t *ch, unsigned char number);
static void clearBuffers(void);
static uint32_t rawBytesAvailable();
static uint32_t bytesAvailable();


static volatile uint32_t usart_periph;
static dma_channel_enum dma_rx_channel, dma_tx_channel;

UART_Config UART =
{
    .mode = UART_MODE_DUAL_WIRE_PushPull,
    .pinout = UART_PINS_DIO17_18,
    .timeout = UART_DEFAULT_TIMEOUT_VALUE,
    .rxtx =
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
    },
    .hideSingleWireEcho = false,
};


static void init()
{
    dma_subperipheral_enum dma_rx_subperipheral, dma_tx_subperipheral;

    // Reset DMA buffer counters
    uartRXReadIndex = 0;
    uartRXEchoBytes = 0;

    switch(UART.pinout) {
    case UART_PINS_DIO10_11:
        //Set MUX_1 and MUX_2 to zero to connect DIO10 and DIO11 to UART pins DIO10_UART_TX and DIO11_UART_RX respectively.
        *HAL.IOs->pins->SW_UART_PWM.resetBitRegister     = HAL.IOs->pins->SW_UART_PWM.bitWeight;

        dma_rx_channel = UART3_DMA_RX_CHANNEL;
        dma_tx_channel = UART3_DMA_TX_CHANNEL;
        dma_rx_subperipheral = UART3_DMA_RX_SUBPERIPH;
        dma_tx_subperipheral = UART3_DMA_TX_SUBPERIPH;

        usart_periph = UART3;
        usart_deinit(usart_periph);

        // Configure the UART pins for operation
        UART_setEnabled(&UART, true);

        rcu_periph_clock_enable(RCU_UART3);
        break;
    case UART_PINS_DIO17_18:
        dma_rx_channel = UART2_DMA_RX_CHANNEL;
        dma_tx_channel = UART2_DMA_TX_CHANNEL;
        dma_rx_subperipheral = UART2_DMA_RX_SUBPERIPH;
        dma_tx_subperipheral = UART2_DMA_TX_SUBPERIPH;

        usart_periph = USART2;
        usart_deinit(usart_periph);

        // Configure the UART pins for operation
        UART_setEnabled(&UART, true);

        rcu_periph_clock_enable(RCU_USART2);
        break;
    }

    uint32_t uclk;
    switch (usart_periph)
    {
    case USART0:
    case USART5:
        uclk = rcu_clock_freq_get(CK_APB2);
        break;
    default:
        uclk = rcu_clock_freq_get(CK_APB1);
        break;
    }

    usart_oversample_config(usart_periph, (UART.rxtx.baudRate > (uclk/16))? USART_OVSMOD_8 : USART_OVSMOD_16);
    usart_baudrate_set(usart_periph, UART.rxtx.baudRate);
    usart_word_length_set(usart_periph, USART_WL_8BIT);
    usart_stop_bit_set(usart_periph, USART_STB_1BIT);
    usart_parity_config(usart_periph, USART_PM_NONE);
    usart_receive_config(usart_periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(usart_periph, USART_TRANSMIT_ENABLE);
    usart_dma_receive_config(usart_periph, USART_DENR_ENABLE);
    usart_dma_transmit_config(usart_periph, USART_DENT_ENABLE);
    usart_enable(usart_periph);

    usart_flag_clear(usart_periph, USART_FLAG_CTS);
    usart_flag_clear(usart_periph, USART_FLAG_LBD);
    usart_flag_clear(usart_periph, USART_FLAG_TBE);
    usart_flag_clear(usart_periph, USART_FLAG_TC);
    usart_flag_clear(usart_periph, USART_FLAG_RBNE);
    usart_flag_clear(usart_periph, USART_FLAG_IDLE);
    usart_flag_clear(usart_periph, USART_FLAG_ORERR);
    usart_flag_clear(usart_periph, USART_FLAG_NERR);
    usart_flag_clear(usart_periph, USART_FLAG_FERR);
    usart_flag_clear(usart_periph, USART_FLAG_PERR);

    // --- DMA configuration ---
    rcu_periph_clock_enable(RCU_DMA0);

    // DMA: RX direction
    // For receiving, we continuously have the DMA read any incoming bytes
    // into the RX circular buffer.
    // Note: If the circular buffer isn't drained fast enough, old bytes will be lost!
    dma_deinit(DMA0, dma_rx_channel);
    dma_single_data_parameter_struct dmaRXConfig;
    dmaRXConfig.periph_addr         = (uint32_t) &USART_DATA(usart_periph);
    dmaRXConfig.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dmaRXConfig.memory0_addr        = (uint32_t) &uartRXDMABuffer[0];
    dmaRXConfig.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dmaRXConfig.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dmaRXConfig.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dmaRXConfig.direction           = DMA_PERIPH_TO_MEMORY;
    dmaRXConfig.number              = UART_RX_DMA_BUFFER_SIZE;
    dmaRXConfig.priority            = DMA_PRIORITY_LOW;
    dma_single_data_mode_init(DMA0, dma_rx_channel, &dmaRXConfig);
    dma_channel_subperipheral_select(DMA0, dma_rx_channel, dma_rx_subperipheral); // DMA0 Channel 1: USART2_RX
    dma_channel_enable(DMA0, dma_rx_channel);

    // DMA: TX direction
    // For transmitting, every time we have data to send, we move it to the DMA buffer,
    // then activate the DMA. If further write requests come in before the DMA is done,
    // or if there is more data than the buffer size, we must wait for the DMA to complete.
    dma_single_data_parameter_struct dmaTXConfig;
    dmaTXConfig.periph_addr         = (uint32_t) &USART_DATA(usart_periph);
    dmaTXConfig.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dmaTXConfig.memory0_addr        = (uint32_t) &uartTXDMABuffer[0];
    dmaTXConfig.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dmaTXConfig.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dmaTXConfig.circular_mode       = DMA_CIRCULAR_MODE_DISABLE;
    dmaTXConfig.direction           = DMA_MEMORY_TO_PERIPH;
    dmaTXConfig.number              = 0;
    dmaTXConfig.priority            = DMA_PRIORITY_LOW;
    dma_single_data_mode_init(DMA0, dma_tx_channel, &dmaTXConfig);
    dma_channel_subperipheral_select(DMA0, dma_tx_channel, dma_tx_subperipheral);
}

static void deInit()
{
    usart_disable(usart_periph);
    dma_channel_disable(DMA0, dma_rx_channel);
    dma_channel_disable(DMA0, dma_tx_channel);

    usart_flag_clear(usart_periph, USART_FLAG_CTS);
    usart_flag_clear(usart_periph, USART_FLAG_LBD);
    usart_flag_clear(usart_periph, USART_FLAG_TBE);
    usart_flag_clear(usart_periph, USART_FLAG_TC);
    usart_flag_clear(usart_periph, USART_FLAG_RBNE);
    usart_flag_clear(usart_periph, USART_FLAG_IDLE);
    usart_flag_clear(usart_periph, USART_FLAG_ORERR);
    usart_flag_clear(usart_periph, USART_FLAG_NERR);
    usart_flag_clear(usart_periph, USART_FLAG_FERR);
    usart_flag_clear(usart_periph, USART_FLAG_PERR);

    clearBuffers();
}

int32_t UART_readWrite(UART_Config *uart, uint8_t *data, size_t writeLength, uint8_t readLength)
{
    uart->rxtx.clearBuffers();
    uart->rxtx.txN(data, writeLength);
    /* Workaround: Give the UART time to send. Otherwise another write/readRegister can do clearBuffers()
     * before we're done. This currently is an issue with the IDE when using the Register browser and the
     * periodic refresh of values gets requested right after the write request.
     */
    wait(2);

    // Abort early if no data needs to be read back
    if (readLength <= 0)
        return 0;

    // Wait for reply with timeout limit
    uint32_t timestamp = systick_getTick();
    while(uart->rxtx.bytesAvailable() < readLength)
    {
        if(timeSince(timestamp) > uart->timeout)
        {
            // Abort on timeout
            return -1;
        }
    }

    uart->rxtx.rxN(data, readLength);

    return 0;
}

void UART_setEnabled(UART_Config *channel, uint8_t enabled)
{
    switch(channel->pinout)
    {
    case UART_PINS_DIO10_11:
        if (enabled)
        {
            // Set the UART alternate function
            gpio_af_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_AF_8, HAL.IOs->pins->DIO10_UART_TX.bitWeight);
            gpio_af_set(HAL.IOs->pins->DIO11_UART_RX.port, GPIO_AF_8, HAL.IOs->pins->DIO11_UART_RX.bitWeight);

            if (channel->mode == UART_MODE_DUAL_WIRE_PushPull)
            {
                // TxD as push-pull output
                gpio_mode_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_MODE_AF, GPIO_PUPD_NONE, HAL.IOs->pins->DIO10_UART_TX.bitWeight);
                gpio_output_options_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO10_UART_TX.bitWeight);
            }
            else
            {
                // TxD as open-drain output with pullup resistor
                gpio_mode_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HAL.IOs->pins->DIO10_UART_TX.bitWeight);
                gpio_output_options_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO10_UART_TX.bitWeight);
            }

            // RxD with pull-up resistor
            gpio_mode_set(HAL.IOs->pins->DIO11_UART_RX.port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HAL.IOs->pins->DIO11_UART_RX.bitWeight);
        }
        else
        {
            // Reset the IOs to their default (input) state
            HAL.IOs->config->reset(&HAL.IOs->pins->DIO10_UART_TX);
            HAL.IOs->config->reset(&HAL.IOs->pins->DIO11_UART_RX);
        }
        break;
    case UART_PINS_DIO17_18:
        if (enabled)
        {
            // Set the UART alternate function
            gpio_af_set(HAL.IOs->pins->DIO17.port, GPIO_AF_7, HAL.IOs->pins->DIO17.bitWeight);
            gpio_af_set(HAL.IOs->pins->DIO18.port, GPIO_AF_7, HAL.IOs->pins->DIO18.bitWeight);

            if (channel->mode == UART_MODE_DUAL_WIRE_PushPull)
            {
                // TxD as push-pull output
                gpio_mode_set(HAL.IOs->pins->DIO17.port, GPIO_MODE_AF, GPIO_PUPD_NONE, HAL.IOs->pins->DIO17.bitWeight);
                gpio_output_options_set(HAL.IOs->pins->DIO17.port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO17.bitWeight);
            }
            else
            {
                // TxD as open-drain output with pullup resistor
                gpio_mode_set(HAL.IOs->pins->DIO17.port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HAL.IOs->pins->DIO17.bitWeight);
                gpio_output_options_set(HAL.IOs->pins->DIO17.port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO17.bitWeight);
            }

            // RxD with pull-up resistor
            gpio_mode_set(HAL.IOs->pins->DIO18.port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HAL.IOs->pins->DIO18.bitWeight);
        }
        else
        {
            // Reset the IOs to their default (input) state
            HAL.IOs->config->reset(&HAL.IOs->pins->DIO17);
            HAL.IOs->config->reset(&HAL.IOs->pins->DIO18);
        }
        break;
    }
}

void UART_flushWriteBuffer(UART_Config *channel)
{
    UNUSED(channel);

    // The write buffer is in the DMA layer
    // Wait until DMA is done first
    if (DMA_CHCTL(DMA0, dma_tx_channel) & DMA_CHXCTL_CHEN)
    {
        while (!dma_flag_get(DMA0, dma_tx_channel, DMA_FLAG_FTF));
    }

    // Then wait for the transmission to be complete
    while (!usart_flag_get(usart_periph, USART_FLAG_TC));
}

static void tx(uint8_t ch)
{
    txN(&ch, 1);
}

static uint8_t rx(uint8_t *ch)
{
    return rxN(ch, 1);
}

static void txN(uint8_t *str, unsigned char number)
{
    while (number > 0)
    {
        // If DMA is active, wait until DMA is done
        if (DMA_CHCTL(DMA0, dma_tx_channel) & DMA_CHXCTL_CHEN)
        {
            while (!dma_flag_get(DMA0, dma_tx_channel, DMA_FLAG_FTF));
        }

        // Clear the DMA Full transfer complete flag
        dma_flag_clear(DMA0, dma_tx_channel, DMA_FLAG_FTF);

        // Move data into DMA buffer
        uint32_t byteCount = MIN(number, sizeof(uartTXDMABuffer));
        memcpy(&uartTXDMABuffer[0], str, byteCount);
        str += byteCount;
        number -= byteCount;

        if (UART.hideSingleWireEcho)
        {
            // Store how many bytes are being sent to suppress their
            // echo on the receive path
            uartRXEchoBytes += byteCount;
        }

        // Clear the Transmission complete flag
        usart_flag_clear(usart_periph, USART_FLAG_TC);

        // Trigger DMA transfer
        DMA_CHCNT(DMA0, dma_tx_channel) = byteCount;
        dma_channel_enable(DMA0, dma_tx_channel);
    }
}

static uint8_t rxN(uint8_t *str, unsigned char number)
{
    uint32_t available = rawBytesAvailable();

    // Skip past echo bytes
    if (UART.hideSingleWireEcho && uartRXEchoBytes > 0)
    {
        uint32_t skipCount = MIN(available, uartRXEchoBytes);

        uartRXReadIndex  = (uartRXReadIndex + skipCount) % UART_RX_DMA_BUFFER_SIZE;
        uartRXEchoBytes -= skipCount;
    }

    if(bytesAvailable() < number)
        return 0;

    // Load data from DMA buffer
    // Skip any remaining echo bytes
    uint32_t readIndex = (uartRXReadIndex + uartRXEchoBytes) % UART_RX_DMA_BUFFER_SIZE;

    // Make sure the copy doesn't overflow past the DMA buffer
    uint32_t bytesUntilWraparound = UART_RX_DMA_BUFFER_SIZE - readIndex;
    uint32_t byteCount = MIN(number, bytesUntilWraparound);
    memcpy(str, &uartRXDMABuffer[readIndex], byteCount);
    if (byteCount < number)
    {
        // If we didn't copy all bytes yet, the read wraps around the
        // buffer. Do a second memcpy from the start of the buffer
        // to grab the remaining bytes
        memcpy(&str[byteCount], &uartRXDMABuffer[0], number-byteCount);
    }

    // Move the read index
    uartRXReadIndex = (readIndex + number) % UART_RX_DMA_BUFFER_SIZE;

    return 1;
}

static void clearBuffers(void)
{
    __disable_irq();
    // Set the UART read index equal to the write index
    uartRXReadIndex = UART_RX_DMA_BUFFER_SIZE - dma_transfer_number_get(DMA0, dma_rx_channel);

    // Reset the echo counter
    uartRXEchoBytes = 0;

    // Stop the transmission
    dma_channel_disable(DMA0, dma_tx_channel);

    // Ensure the receiver is enabled again
    usart_receive_config(usart_periph, USART_RECEIVE_ENABLE);
    __enable_irq();
}

// Total bytes available including echo bytes
static uint32_t rawBytesAvailable()
{
    uint32_t dmaCounter = dma_transfer_number_get(DMA0, dma_rx_channel);
    uint32_t uartRXWriteIndex = UART_RX_DMA_BUFFER_SIZE - dmaCounter;

    return (uartRXWriteIndex - uartRXReadIndex + UART_RX_DMA_BUFFER_SIZE) % UART_RX_DMA_BUFFER_SIZE;
}

// Bytes available - excluding echo bytes if echo suppression is enabled
static uint32_t bytesAvailable()
{
    uint32_t bytes = rawBytesAvailable();
    if (bytes < uartRXEchoBytes)
        return 0;

    return bytes - uartRXEchoBytes;
}
