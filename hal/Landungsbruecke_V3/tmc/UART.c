/*******************************************************************************
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "hal/HAL.h"
#include "hal/UART.h"

#define BUFFER_SIZE  1024
#define INTR_PRI     6
#define UART_DEFAULT_TIMEOUT_VALUE 10 // [ms]

static void init();
static void deInit();
static void tx(uint8_t ch);
static uint8_t rx(uint8_t *ch);
static void txN(uint8_t *str, unsigned char number);
static uint8_t rxN(uint8_t *ch, unsigned char number);
static void clearBuffers(void);
static uint32_t bytesAvailable();

static uint8_t UARTSendFlag;

static volatile uint8_t rxBuffer[BUFFER_SIZE];
static volatile uint8_t txBuffer[BUFFER_SIZE];
static volatile uint32_t usart_periph;
uint8_t volatile nvic_irq;

static volatile uint32_t available = 0;

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

void __attribute__ ((interrupt)) USART2_IRQHandler(void);
void __attribute__ ((interrupt)) UART3_IRQHandler(void);


static void init()
{

    switch(UART.pinout) {
    case UART_PINS_2:
        //Set MUX_1 and MUX_2 to zero to connect DIO10 and DIO11 to UART pins DIO10_UART_TX and DIO11_UART_RX respectively.
        *HAL.IOs->pins->SW_UART_PWM.resetBitRegister     = HAL.IOs->pins->SW_UART_PWM.bitWeight;

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
    usart_enable(usart_periph);
    usart_interrupt_enable(usart_periph, USART_INT_TBE);
    usart_interrupt_enable(usart_periph, USART_INT_TC);
    usart_interrupt_enable(usart_periph, USART_INT_RBNE);

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

}

static void deInit()
{
    usart_disable(usart_periph);
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
    uint8_t byte;
    usart_periph = USART2;
    // Receive interrupt
    if(USART_STAT0(usart_periph) & USART_STAT0_RBNE)
    {
        // One-wire UART communication:
        // Ignore received byte when a byte has just been sent (echo).
        byte = USART_DATA(usart_periph);

        if(!UARTSendFlag)
        {
            // not sending, received real data instead of echo -> advance ring buffer index and available counter
            buffers.rx.buffer[buffers.rx.wrote]=byte;
            buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
            available++;
        }
    }

    // Transmit buffer empty interrupt => send next byte if there is something
    // to be sent.
    if(USART_STAT0(usart_periph) & USART_STAT0_TBE)
    {
        if(buffers.tx.read != buffers.tx.wrote)
        {
            UARTSendFlag = true;
            USART_DATA(usart_periph)  = buffers.tx.buffer[buffers.tx.read];
            buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;
        }
        else
        {
            usart_interrupt_disable(usart_periph, USART_INT_TBE);
        }
    }

    // Transmission complete interrupt => do not ignore echo any more
    // after last bit has been sent.
    if(USART_STAT0(usart_periph) & USART_STAT0_TC)
    {
        //Only if there are no more bytes left in the transmit buffer
        if(buffers.tx.read == buffers.tx.wrote)
        {
            byte = USART_DATA(usart_periph);  //Ignore spurios echos of the last sent byte that sometimes occur.
            UARTSendFlag = false;
        }

        //USART_ClearITPendingBit(USART2, USART_IT_TC);
        usart_interrupt_flag_clear(usart_periph, USART_INT_FLAG_TC);
    }
}

void UART3_IRQHandler(void)
{
    uint8_t byte;
    usart_periph = UART3;
    // Receive interrupt
    if(USART_STAT0(usart_periph) & USART_STAT0_RBNE)
    {
        // One-wire UART communication:
        // Ignore received byte when a byte has just been sent (echo).
        byte = USART_DATA(usart_periph);

        if(!UARTSendFlag)
        {
            // not sending, received real data instead of echo -> advance ring buffer index and available counter
            buffers.rx.buffer[buffers.rx.wrote]=byte;
            buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
            available++;
        }
    }

    // Transmit buffer empty interrupt => send next byte if there is something
    // to be sent.
    if(USART_STAT0(usart_periph) & USART_STAT0_TBE)
    {
        if(buffers.tx.read != buffers.tx.wrote)
        {
            UARTSendFlag = true;
            USART_DATA(usart_periph)  = buffers.tx.buffer[buffers.tx.read];
            buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;
        }
        else
        {
            usart_interrupt_disable(usart_periph, USART_INT_TBE);
        }
    }

    // Transmission complete interrupt => do not ignore echo any more
    // after last bit has been sent.
    if(USART_STAT0(usart_periph) & USART_STAT0_TC)
    {
        //Only if there are no more bytes left in the transmit buffer
        if(buffers.tx.read == buffers.tx.wrote)
        {
            byte = USART_DATA(usart_periph);  //Ignore spurios echos of the last sent byte that sometimes occur.
            UARTSendFlag = false;
        }

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
    buffers.tx.buffer[buffers.tx.wrote] = ch;
    buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;
    usart_interrupt_enable(usart_periph, USART_INT_TBE);

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

