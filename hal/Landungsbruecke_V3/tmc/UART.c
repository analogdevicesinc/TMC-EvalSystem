/*******************************************************************************
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
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



static void init();
static void deInit();
static void tx(uint8_t ch);
static uint8_t rx(uint8_t *ch);
static void txN(uint8_t *str, unsigned char number);
static uint8_t rxN(uint8_t *ch, unsigned char number);
static void clearBuffers(void);
static uint32_t bytesAvailable();


static volatile uint32_t usart_periph;
static dma_channel_enum dma_rx_channel, dma_tx_channel;

uint8_t volatile nvic_irq;

UART_Config UART =
{
    .mode = UART_MODE_DUAL_WIRE,
    .pinout = UART_PINS_1,
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
    }
};

void __attribute__ ((interrupt)) USART2_IRQHandler(void);
void __attribute__ ((interrupt)) UART3_IRQHandler(void);


static void init()
{
    dma_subperipheral_enum dma_rx_subperipheral, dma_tx_subperipheral;

    switch(UART.pinout) {
    case UART_PINS_2:
        //Set MUX_1 and MUX_2 to zero to connect DIO10 and DIO11 to UART pins DIO10_UART_TX and DIO11_UART_RX respectively.
        *HAL.IOs->pins->SW_UART_PWM.resetBitRegister     = HAL.IOs->pins->SW_UART_PWM.bitWeight;

        dma_rx_channel = UART3_DMA_RX_CHANNEL;
        dma_tx_channel = UART3_DMA_TX_CHANNEL;
        dma_rx_subperipheral = UART3_DMA_RX_SUBPERIPH;
        dma_tx_subperipheral = UART3_DMA_TX_SUBPERIPH;

        usart_periph = UART3;
        usart_deinit(usart_periph);
        //DIO10_UART_TX(TxD) with pull-up resistor
        gpio_mode_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_MODE_AF, GPIO_PUPD_NONE, HAL.IOs->pins->DIO10_UART_TX.bitWeight);
        gpio_output_options_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO10_UART_TX.bitWeight);

        //DIO11_UART_RX(RxD) with pull-up resistor
        gpio_mode_set(HAL.IOs->pins->DIO11_UART_RX.port, GPIO_MODE_AF, GPIO_PUPD_NONE, HAL.IOs->pins->DIO11_UART_RX.bitWeight);
        gpio_output_options_set(HAL.IOs->pins->DIO11_UART_RX.port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO11_UART_RX.bitWeight);

        gpio_af_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_AF_8, HAL.IOs->pins->DIO10_UART_TX.bitWeight);
        gpio_af_set(HAL.IOs->pins->DIO11_UART_RX.port, GPIO_AF_8, HAL.IOs->pins->DIO11_UART_RX.bitWeight);
        rcu_periph_clock_enable(RCU_UART3);
        nvic_irq = UART3_IRQn;
        break;
    case UART_PINS_1:
        dma_rx_channel = UART2_DMA_RX_CHANNEL;
        dma_tx_channel = UART2_DMA_TX_CHANNEL;
        dma_rx_subperipheral = UART2_DMA_RX_SUBPERIPH;
        dma_tx_subperipheral = UART2_DMA_TX_SUBPERIPH;

        usart_periph = USART2;
        usart_deinit(usart_periph);
        //DIO17(TxD) with pull-up resistor
        gpio_mode_set(HAL.IOs->pins->DIO17.port, GPIO_MODE_AF, GPIO_PUPD_NONE, HAL.IOs->pins->DIO17.bitWeight);
        gpio_output_options_set(HAL.IOs->pins->DIO17.port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO17.bitWeight);

        //DIO18(RxD) with pull-up resistor
        gpio_mode_set(HAL.IOs->pins->DIO18.port, GPIO_MODE_AF, GPIO_PUPD_NONE, HAL.IOs->pins->DIO18.bitWeight);
        gpio_output_options_set(HAL.IOs->pins->DIO18.port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO18.bitWeight);

        gpio_af_set(HAL.IOs->pins->DIO17.port, GPIO_AF_7, HAL.IOs->pins->DIO17.bitWeight);
        gpio_af_set(HAL.IOs->pins->DIO18.port, GPIO_AF_7, HAL.IOs->pins->DIO18.bitWeight);
        rcu_periph_clock_enable(RCU_USART2);
        usart_hardware_flow_rts_config(usart_periph, USART_RTS_DISABLE);
        usart_hardware_flow_cts_config(usart_periph, USART_CTS_DISABLE);
        nvic_irq = USART2_IRQn;
        break;
    }


    usart_baudrate_set(usart_periph, UART.rxtx.baudRate);
    usart_word_length_set(usart_periph, USART_WL_8BIT);
    usart_stop_bit_set(usart_periph, USART_STB_1BIT);
    usart_parity_config(usart_periph, USART_PM_NONE);
    usart_receive_config(usart_periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(usart_periph, USART_TRANSMIT_ENABLE);
    usart_dma_receive_config(usart_periph, USART_DENR_ENABLE);
    usart_dma_transmit_config(usart_periph, USART_DENT_ENABLE);
    usart_enable(usart_periph);
    usart_interrupt_enable(usart_periph, USART_INT_TC);

    nvic_irq_enable(nvic_irq, INTR_PRI, 0);

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

    nvic_irq_disable(nvic_irq);

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

void USART2_IRQHandler(void)
{
    usart_periph = USART2;

    // Transmission complete interrupt => do not ignore echo any more
    // after last bit has been sent.
    if(USART_STAT0(usart_periph) & USART_STAT0_TC)
    {
        // Re-enable the receiver to receive replies after requests
        usart_receive_config(usart_periph, USART_RECEIVE_ENABLE);

        //USART_ClearITPendingBit(USART2, USART_IT_TC);
        usart_interrupt_flag_clear(usart_periph, USART_INT_FLAG_TC);
    }
}

void UART3_IRQHandler(void)
{
    usart_periph = UART3;

    // Transmission complete interrupt => do not ignore echo any more
    // after last bit has been sent.
    if(USART_STAT0(usart_periph) & USART_STAT0_TC)
    {
        // Re-enable the receiver to receive replies after requests
        usart_receive_config(usart_periph, USART_RECEIVE_ENABLE);

        //USART_ClearITPendingBit(USART2, USART_IT_TC);
        usart_interrupt_flag_clear(usart_periph, USART_INT_FLAG_TC);
    }
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

void UART_readInt(UART_Config *channel, uint8_t slave, uint8_t address, int32_t *value)
{
    uint8_t readData[8], dataRequest[4];
    uint32_t timeout;

    dataRequest[0] = 0x05;                        // Sync byte
    dataRequest[1] = slave;                       // Slave address
    dataRequest[2] = address;                     // Register address
    dataRequest[3] = tmc_CRC8(dataRequest, 3, 1); // Cyclic redundancy check

    channel->rxtx.clearBuffers();
    channel->rxtx.txN(dataRequest, ARRAY_SIZE(dataRequest));

    // Wait for reply with timeout limit
    timeout = systick_getTick();
    while(channel->rxtx.bytesAvailable() < ARRAY_SIZE(readData))
        if(timeSince(timeout) > channel->timeout) // Timeout
            return;

    channel->rxtx.rxN(readData, ARRAY_SIZE(readData));
    // Check if the received data is correct (CRC, Sync, Slave address, Register address)
    // todo CHECK 2: Only keep CRC check? Should be sufficient for wrong transmissions (LH) #1
    if(readData[7] != tmc_CRC8(readData, 7, 1) || readData[0] != 0x05 || readData[1] != 0xFF || readData[2] != address)
        return;

    *value = readData[3] << 24 | readData[4] << 16 | readData[5] << 8 | readData[6];
    return;
}

void UART_writeInt(UART_Config *channel, uint8_t slave, uint8_t address, int32_t value)
{
    uint8_t writeData[8];

    writeData[0] = 0x05;                         // Sync byte
    writeData[1] = slave;                        // Slave address
    writeData[2] = address | TMC_WRITE_BIT;      // Register address with write bit set
    writeData[3] = value >> 24;                  // Register Data
    writeData[4] = value >> 16;                  // Register Data
    writeData[5] = value >> 8;                   // Register Data
    writeData[6] = value & 0xFF;                 // Register Data
    writeData[7] = tmc_CRC8(writeData, 7, 1);    // Cyclic redundancy check

    channel->rxtx.clearBuffers();
    for(uint32_t i = 0; i < ARRAY_SIZE(writeData); i++)
        channel->rxtx.tx(writeData[i]);

    /* Workaround: Give the UART time to send. Otherwise another write/readRegister can do clearBuffers()
     * before we're done. This currently is an issue with the IDE when using the Register browser and the
     * periodic refresh of values gets requested right after the write request.
     */
    wait(2);
}

void UART_setEnabled(UART_Config *channel, uint8_t enabled)
{
    UNUSED(channel);
    switch(channel->pinout)
    {
    case UART_PINS_2:
        if (enabled)
        {

            //TxD as open drain output
            gpio_mode_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_MODE_AF, GPIO_PUPD_NONE, HAL.IOs->pins->DIO10_UART_TX.bitWeight);
            gpio_output_options_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO10_UART_TX.bitWeight);

            //RxD with pull-up resistor
            gpio_mode_set(HAL.IOs->pins->DIO11_UART_RX.port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HAL.IOs->pins->DIO11_UART_RX.bitWeight);
            gpio_output_options_set(HAL.IOs->pins->DIO11_UART_RX.port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO11_UART_RX.bitWeight);

            gpio_af_set(HAL.IOs->pins->DIO10_UART_TX.port, GPIO_AF_8, HAL.IOs->pins->DIO10_UART_TX.bitWeight);
            gpio_af_set(HAL.IOs->pins->DIO11_UART_RX.port, GPIO_AF_8, HAL.IOs->pins->DIO11_UART_RX.bitWeight);
        }
        else{
            HAL.IOs->config->reset(&HAL.IOs->pins->DIO10_UART_TX);
            HAL.IOs->config->reset(&HAL.IOs->pins->DIO11_UART_RX);
        }
        break;
    case UART_PINS_1:
        if (enabled)
        {
            //TxD as open drain output
            gpio_mode_set(HAL.IOs->pins->DIO17.port, GPIO_MODE_AF, GPIO_PUPD_NONE, HAL.IOs->pins->DIO17.bitWeight);
            gpio_output_options_set(HAL.IOs->pins->DIO17.port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO17.bitWeight);

            //RxD with pull-up resistor
            gpio_mode_set(HAL.IOs->pins->DIO18.port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HAL.IOs->pins->DIO18.bitWeight);
            gpio_output_options_set(HAL.IOs->pins->DIO18.port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, HAL.IOs->pins->DIO18.bitWeight);

            gpio_af_set(HAL.IOs->pins->DIO17.port, GPIO_AF_7, HAL.IOs->pins->DIO17.bitWeight);
            gpio_af_set(HAL.IOs->pins->DIO18.port, GPIO_AF_7, HAL.IOs->pins->DIO18.bitWeight);
        }
        else{
            HAL.IOs->config->reset(&HAL.IOs->pins->DIO17);
            HAL.IOs->config->reset(&HAL.IOs->pins->DIO18);
        }
        break;
    }
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

        // Disable the receiver while sending - this hides
        // the echo of single-wire transmission.
        // ToDo: Do this better
        usart_receive_config(usart_periph, USART_RECEIVE_DISABLE);

        // Trigger DMA transfer
        DMA_CHCNT(DMA0, dma_tx_channel) = byteCount;
        dma_channel_enable(DMA0, dma_tx_channel);
    }
}

static uint8_t rxN(uint8_t *str, unsigned char number)
{
    if(bytesAvailable() < number)
        return 0;

    // Load data from DMA buffer
    // Copy at most until the end of the buffer
    uint32_t bytesUntilWraparound = UART_RX_DMA_BUFFER_SIZE - uartRXReadIndex;
    uint32_t byteCount = MIN(number, bytesUntilWraparound);
    memcpy(str, &uartRXDMABuffer[uartRXReadIndex], byteCount);
    if (byteCount < number)
    {
        // If we didn't copy all bytes yet, the read wraps around the
        // buffer. Do a second memcpy from the start of the buffer.
        memcpy(&str[byteCount], &uartRXDMABuffer[0], number-byteCount);
    }

    // Move the read index
    uartRXReadIndex = (uartRXReadIndex + number) % UART_RX_DMA_BUFFER_SIZE;

    return 1;
}

static void clearBuffers(void)
{
    __disable_irq();
    // Set the UART read index equal to the write index
    uartRXReadIndex = UART_RX_DMA_BUFFER_SIZE - dma_transfer_number_get(DMA0, dma_rx_channel);

    // Stop the transmission
    dma_channel_disable(DMA0, dma_tx_channel);

    // Ensure the receiver is enabled again
    usart_receive_config(usart_periph, USART_RECEIVE_ENABLE);
    __enable_irq();
}

static uint32_t bytesAvailable()
{
    uint32_t dmaCounter = dma_transfer_number_get(DMA0, dma_rx_channel);
    uint32_t uartRXWriteIndex = UART_RX_DMA_BUFFER_SIZE - dmaCounter;

    return (uartRXWriteIndex - uartRXReadIndex + UART_RX_DMA_BUFFER_SIZE) % UART_RX_DMA_BUFFER_SIZE;
}

