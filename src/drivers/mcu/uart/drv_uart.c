/*
	ATMEGA uart common layer.
	- buffered uart io.
	- with RXC,TXC,UDRE interrupt
	
	@ver 1.1

	2015/12/04 interrupt e/d updated, var updated.
*/
#include "avr/io.h"
#include "avr/interrupt.h"

#include "putil.h"
#include "uart.h"
#include "drv_uart.h"
#include <string.h>
#include "../dma/drv_dma.h"

#define	UART_BUFF_SIZE		128

static uint8	_rx_buff[UART_BUFF_SIZE];
static uint8	_rx_front = 0;
static uint8	_rx_tail = 0;
static uint8	_rx_len = 0;

static uint8	_tx_buff[UART_BUFF_SIZE];
static uint8	_tx_front = 0;
static uint8	_tx_tail = 0;
static uint8	_tx_len = 0;

// Default buffer sizes if not specified in config
#define DEFAULT_RX_BUFFER_SIZE 64
#define DEFAULT_TX_BUFFER_SIZE 64

// Maximum number of UART ports
#define MAX_UART_PORTS UART_PORT_MAX

// UART hardware register maps
static const uart_registers_t uart_registers[MAX_UART_PORTS] = {
    // UART1
    {
        .UDRn = &UDR1,
        .UCSRnA = &UCSR1A,
        .UCSRnB = &UCSR1B,
        .UCSRnC = &UCSR1C,
        .UBRRnH = &UBRR1H,
        .UBRRnL = &UBRR1L,
        .RXCIEn = (1 << RXCIE1),
        .TXCIEn = (1 << TXCIE1),
        .UDRIEn = (1 << UDRIE1)
    },
    // UART2
    {
        .UDRn = &UDR2,
        .UCSRnA = &UCSR2A,
        .UCSRnB = &UCSR2B,
        .UCSRnC = &UCSR2C,
        .UBRRnH = &UBRR2H,
        .UBRRnL = &UBRR2L,
        .RXCIEn = (1 << RXCIE2),
        .TXCIEn = (1 << TXCIE2),
        .UDRIEn = (1 << UDRIE2)
    },
    // UART3
    {
        .UDRn = &UDR3,
        .UCSRnA = &UCSR3A,
        .UCSRnB = &UCSR3B,
        .UCSRnC = &UCSR3C,
        .UBRRnH = &UBRR3H,
        .UBRRnL = &UBRR3L,
        .RXCIEn = (1 << RXCIE3),
        .TXCIEn = (1 << TXCIE3),
        .UDRIEn = (1 << UDRIE3)
    }
};

// UART instance data
typedef struct {
    uart_config_t config;
    uart_status_t status;
    uart_driver_t driver;
    ring_buffer_t rx_buffer;
    ring_buffer_t tx_buffer;
    uint8_t *rx_buffer_data;
    uint8_t *tx_buffer_data;
    bool initialized;
} uart_instance_t;

static uart_instance_t uart_instances[MAX_UART_PORTS];

// Forward declarations of static functions
static error_code_t uart_configure_port(uart_port_t port, const uart_config_t* config);
static void uart_update_status(uart_port_t port, error_code_t error);

// Initialize UART port
error_code_t uart_init(uart_port_t port, const uart_config_t* config) {
    if (port >= MAX_UART_PORTS) {
        return ERR_UART_INVALID_PORT;
    }
    
    if (!config) {
        return ERR_INVALID_PARAM;
    }
    
    // Validate configuration parameters
    if (config->baud_rate == 0 || 
        config->data_bits < 5 || config->data_bits > 8 ||
        config->stop_bits < 1 || config->stop_bits > 2) {
        return ERR_INVALID_PARAM;
    }
    
    // Calculate baud rate register value and verify it's within acceptable range
    uint32_t baud_setting = (F_CPU / 8 / config->baud_rate - 1);
    if (baud_setting > 0xFFF) {  // 12-bit max for baud rate generator
        return ERR_INVALID_PARAM;
    }
    
    uart_instance_t* inst = &uart_instances[port];
    error_code_t err;
    
    // Allocate buffers if not already allocated
    if (!inst->rx_buffer_data) {
        inst->rx_buffer_data = malloc(config->rx_buffer_size ? 
                                    config->rx_buffer_size : DEFAULT_RX_BUFFER_SIZE);
        if (!inst->rx_buffer_data) {
            return ERR_OUT_OF_MEMORY;
        }
    }
    
    if (!inst->tx_buffer_data) {
        inst->tx_buffer_data = malloc(config->tx_buffer_size ? 
                                    config->tx_buffer_size : DEFAULT_TX_BUFFER_SIZE);
        if (!inst->tx_buffer_data) {
            free(inst->rx_buffer_data);
            inst->rx_buffer_data = NULL;
            return ERR_OUT_OF_MEMORY;
        }
    }
    
    // Initialize ring buffers with atomic operations
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ring_buffer_init(&inst->rx_buffer, inst->rx_buffer_data, 
                        config->rx_buffer_size ? config->rx_buffer_size : DEFAULT_RX_BUFFER_SIZE);
        ring_buffer_init(&inst->tx_buffer, inst->tx_buffer_data,
                        config->tx_buffer_size ? config->tx_buffer_size : DEFAULT_TX_BUFFER_SIZE);
    }
    
    // Store configuration
    memcpy(&inst->config, config, sizeof(uart_config_t));
    
    // Configure hardware with interrupts disabled
    cli();
    err = uart_configure_port(port, config);
    sei();
    
    if (err != ERROR_CODE_SUCCESS) {
        uart_deinit(port);  // Clean up on error
        return err;
    }
    
    // Initialize status
    memset(&inst->status, 0, sizeof(uart_status_t));
    inst->status.state = UART_STATE_ACTIVE;
    
    // Initialize DMA if enabled
    if (config->dma_flags & (UART_FLAG_DMA_TX | UART_FLAG_DMA_RX)) {
        err = uart_init_dma(port, config);
        if (err != ERROR_CODE_SUCCESS) {
            uart_deinit(port);
            return err;
        }
    }
    
    inst->initialized = true;
    return ERROR_CODE_SUCCESS;
}

// Configure UART hardware
static error_code_t uart_configure_port(uart_port_t port, const uart_config_t* config) {
    const uart_registers_t* regs = &uart_registers[port];
    uint8_t ucsr_a = 0;
    uint8_t ucsr_b = 0;
    uint8_t ucsr_c = 0;
    
    // Calculate baud rate register value
    uint16_t baud_setting = (F_CPU / 8 / config->baud_rate - 1);
    
    // Configure UCSRA
    ucsr_a = (1 << U2X0);  // Double speed mode
    
    // Configure UCSRB
    ucsr_b = regs->RXCIEn |                    // RX Complete Interrupt Enable
             (1 << RXEN0) | (1 << TXEN0);      // Enable RX and TX
    
    if (config->dma_flags & UART_FLAG_DMA_TX) {
        ucsr_b &= ~regs->UDRIEn;  // Disable UDRE interrupt for DMA
    }
    
    // Configure UCSRC
    // Set data bits
    switch (config->data_bits) {
        case 5: ucsr_c |= 0; break;
        case 6: ucsr_c |= (1 << UCSZ00); break;
        case 7: ucsr_c |= (1 << UCSZ01); break;
        case 8: ucsr_c |= (3 << UCSZ00); break;
        default: return ERR_INVALID_PARAM;
    }
    
    // Set stop bits
    if (config->stop_bits == 2) {
        ucsr_c |= (1 << USBS0);
    }
    
    // Set parity
    switch (config->parity) {
        case UART_PARITY_NONE: break;
        case UART_PARITY_EVEN: ucsr_c |= (2 << UPM00); break;
        case UART_PARITY_ODD:  ucsr_c |= (3 << UPM00); break;
        default: return ERR_INVALID_PARAM;
    }
    
    // Disable interrupts during configuration
    uint8_t sreg = SREG;
    cli();
    
    // Apply configuration
    *regs->UCSRnA = ucsr_a;
    *regs->UCSRnB = ucsr_b;
    *regs->UCSRnC = ucsr_c;
    
    // Set baud rate
    *regs->UBRRnH = baud_setting >> 8;
    *regs->UBRRnL = baud_setting;
    
    // Restore interrupt state
    SREG = sreg;
    
    // Verify configuration was applied correctly
    if (*regs->UCSRnA != ucsr_a ||
        *regs->UCSRnB != ucsr_b ||
        *regs->UCSRnC != ucsr_c ||
        *regs->UBRRnH != (baud_setting >> 8) ||
        *regs->UBRRnL != (baud_setting & 0xFF)) {
        return ERR_HARDWARE;
    }
    
    return ERROR_CODE_SUCCESS;
}

// Update UART status
static void uart_update_status(uart_port_t port, error_code_t error) {
    uart_instance_t* inst = &uart_instances[port];
    
    // Update error status atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        inst->status.last_error = error;
        
        if (error != ERROR_CODE_SUCCESS) {
            inst->status.error_count++;
            
            // Update specific error counters
            switch (error) {
                case UART_ERR_OVERFLOW:
                    inst->status.overflow_count++;
                    break;
                case UART_ERR_FRAMING:
                    inst->status.framing_errors++;
                    break;
                case UART_ERR_PARITY:
                    inst->status.parity_errors++;
                    break;
                case UART_ERR_NOISE:
                    inst->status.noise_errors++;
                    break;
                case UART_ERR_BREAK:
                    inst->status.break_count++;
                    break;
            }
        }
    }
    
    // Call error callback if registered and error occurred
    if (error != ERROR_CODE_SUCCESS && inst->config.error_callback) {
        inst->config.error_callback(error);
    }
}

// Interrupt handlers for each UART port
ISR(USART1_RX_vect) {
    uint8_t status = UCSR1A;
    uint8_t data = UDR1;
    uart_instance_t* inst = &uart_instances[UART_PORT_1];
    
    // Check for UART errors
    if (status & ((1 << FE1) | (1 << DOR1) | (1 << UPE1))) {
        if (status & (1 << FE1)) {
            uart_update_status(UART_PORT_1, ERR_UART_FRAME_ERROR);
        }
        if (status & (1 << DOR1)) {
            uart_update_status(UART_PORT_1, ERR_UART_OVERFLOW);
        }
        if (status & (1 << UPE1)) {
            uart_update_status(UART_PORT_1, ERR_UART_PARITY_ERROR);
        }
        inst->status.error_count++;
        return;
    }
    
    if (ring_buffer_push(&inst->rx_buffer, data) != ERROR_CODE_SUCCESS) {
        uart_update_status(UART_PORT_1, ERR_UART_OVERFLOW);
        inst->status.overflow_count++;
    } else {
        inst->status.rx_count++;
    }
}

ISR(USART2_RX_vect) {
    uint8_t status = UCSR2A;
    uint8_t data = UDR2;
    uart_instance_t* inst = &uart_instances[UART_PORT_2];
    
    // Check for UART errors
    if (status & ((1 << FE2) | (1 << DOR2) | (1 << UPE2))) {
        if (status & (1 << FE2)) {
            uart_update_status(UART_PORT_2, ERR_UART_FRAME_ERROR);
        }
        if (status & (1 << DOR2)) {
            uart_update_status(UART_PORT_2, ERR_UART_OVERFLOW);
        }
        if (status & (1 << UPE2)) {
            uart_update_status(UART_PORT_2, ERR_UART_PARITY_ERROR);
        }
        inst->status.error_count++;
        return;
    }
    
    if (ring_buffer_push(&inst->rx_buffer, data) != ERROR_CODE_SUCCESS) {
        uart_update_status(UART_PORT_2, ERR_UART_OVERFLOW);
        inst->status.overflow_count++;
    } else {
        inst->status.rx_count++;
    }
}

ISR(USART3_RX_vect) {
    uint8_t status = UCSR3A;
    uint8_t data = UDR3;
    uart_instance_t* inst = &uart_instances[UART_PORT_3];
    
    // Check for UART errors
    if (status & ((1 << FE3) | (1 << DOR3) | (1 << UPE3))) {
        if (status & (1 << FE3)) {
            uart_update_status(UART_PORT_3, ERR_UART_FRAME_ERROR);
        }
        if (status & (1 << DOR3)) {
            uart_update_status(UART_PORT_3, ERR_UART_OVERFLOW);
        }
        if (status & (1 << UPE3)) {
            uart_update_status(UART_PORT_3, ERR_UART_PARITY_ERROR);
        }
        inst->status.error_count++;
        return;
    }
    
    if (ring_buffer_push(&inst->rx_buffer, data) != ERROR_CODE_SUCCESS) {
        uart_update_status(UART_PORT_3, ERR_UART_OVERFLOW);
        inst->status.overflow_count++;
    } else {
        inst->status.rx_count++;
    }
}

ISR(UART_RX_VECT)
{
    uint8_t status = REG_UCSRA;
    uint8_t data;
    
    // Check for errors first
    if (status & ((1<<FE)|(1<<DOR)|(1<<PE))) {
        // Clear error by reading data
        data = REG_UDR;
        if (status & (1<<FE)) error_record(ERR_UART_FRAMING);
        if (status & (1<<DOR)) error_record(ERR_UART_OVERFLOW);
        if (status & (1<<PE)) error_record(ERR_UART_PARITY);
        return;
    }
    
    // Read data
    data = REG_UDR;
    
    // Check for buffer overflow before saving
    if (_rx_len >= UART_BUFF_SIZE) {
        error_record(ERR_UART_BUFFER_OVERFLOW);
        return;
    }
    
    // Save to buffer with atomic operation
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _rx_buff[_rx_tail] = data;
        _rx_tail = (_rx_tail + 1) % UART_BUFF_SIZE;
        _rx_len++;
    }
}

ISR(UART_UDRE_VECT)
{
    uint8_t c;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Check if there's data to transmit
        if (_tx_len > 0) {
            c = _tx_buff[_tx_front];
            REG_UDR = c;
            
            _tx_front = (_tx_front + 1) % UART_BUFF_SIZE;
            _tx_len--;
        }
        
        // Disable UDRE interrupt if no more data
        if (_tx_len == 0) {
            DISABLE_UDRE();
            ENABLE_TXC();  // Enable TXC for next char output
        }
    }
}

ISR(UART_TX_VECT)
{
    uint8_t c;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_tx_len > 0) {
            c = _tx_buff[_tx_front];
            REG_UDR = c;
            
            _tx_front = (_tx_front + 1) % UART_BUFF_SIZE;
            _tx_len--;
            
            if (_tx_len == 0) {
                DISABLE_TXC();  // Disable TX complete interrupt
            }
        }
    }
}

// Implementation of public functions
error_code_t uart_write(uart_port_t port, const uint8_t* data, uint16_t size) {
    if (port >= MAX_UART_PORTS || !uart_instances[port].initialized) {
        return ERR_UART_INVALID_PORT;
    }
    
    uart_instance_t* inst = &uart_instances[port];
    error_code_t err = ERROR_CODE_SUCCESS;
    
    for (uint16_t i = 0; i < size; i++) {
        err = ring_buffer_push(&inst->tx_buffer, data[i]);
        if (err != ERROR_CODE_SUCCESS) {
            uart_update_status(port, err);
            return err;
        }
        inst->status.tx_count++;
    }
    
    // Enable TX interrupt to start transmission
    const uart_registers_t* regs = &uart_registers[port];
    *regs->UCSRnB |= regs->UDRIEn;
    
    return ERROR_CODE_SUCCESS;
}

error_code_t uart_read(uart_port_t port, uint8_t* data, uint16_t size, uint16_t* bytes_read) {
    if (port >= MAX_UART_PORTS || !uart_instances[port].initialized) {
        return ERR_UART_INVALID_PORT;
    }
    
    uart_instance_t* inst = &uart_instances[port];
    uint16_t count = 0;
    error_code_t err;
    
    while (count < size) {
        err = ring_buffer_pop(&inst->rx_buffer, &data[count]);
        if (err != ERROR_CODE_SUCCESS) {
            break;
        }
        count++;
    }
    
    if (bytes_read) {
        *bytes_read = count;
    }
    
    return count > 0 ? ERROR_CODE_SUCCESS : ERR_BUFFER_EMPTY;
}

error_code_t uart_write_byte(uart_port_t port, uint8_t byte) {
    return uart_write(port, &byte, 1);
}

error_code_t uart_read_byte(uart_port_t port, uint8_t* byte) {
    uint16_t bytes_read;
    return uart_read(port, byte, 1, &bytes_read);
}

error_code_t uart_flush_rx(uart_port_t port) {
    if (port >= MAX_UART_PORTS || !uart_instances[port].initialized) {
        return ERR_UART_INVALID_PORT;
    }
    
    uart_instance_t* inst = &uart_instances[port];
    ring_buffer_clear(&inst->rx_buffer);
    return ERROR_CODE_SUCCESS;
}

error_code_t uart_flush_tx(uart_port_t port) {
    if (port >= MAX_UART_PORTS || !uart_instances[port].initialized) {
        return ERR_UART_INVALID_PORT;
    }
    
    uart_instance_t* inst = &uart_instances[port];
    ring_buffer_clear(&inst->tx_buffer);
    return ERROR_CODE_SUCCESS;
}

uint16_t uart_rx_available(uart_port_t port) {
    if (port >= MAX_UART_PORTS || !uart_instances[port].initialized) {
        return 0;
    }
    
    uart_instance_t* inst = &uart_instances[port];
    return ring_buffer_count(&inst->rx_buffer);
}

uint16_t uart_tx_space(uart_port_t port) {
    if (port >= MAX_UART_PORTS || !uart_instances[port].initialized) {
        return 0;
    }
    
    uart_instance_t* inst = &uart_instances[port];
    return inst->tx_buffer.size - ring_buffer_count(&inst->tx_buffer);
}

const uart_status_t* uart_get_status(uart_port_t port) {
    if (port >= MAX_UART_PORTS || !uart_instances[port].initialized) {
        return NULL;
    }
    
    return &uart_instances[port].status;
}

error_code_t uart_deinit(uart_port_t port) {
    if (port >= MAX_UART_PORTS) {
        return ERR_UART_INVALID_PORT;
    }
    
    uart_instance_t* inst = &uart_instances[port];
    if (!inst->initialized) {
        return ERROR_CODE_SUCCESS;
    }
    
    // Disable UART hardware
    const uart_registers_t* regs = &uart_registers[port];
    *regs->UCSRnB = 0;
    
    // Free buffers
    if (inst->rx_buffer_data) {
        free(inst->rx_buffer_data);
        inst->rx_buffer_data = NULL;
    }
    
    if (inst->tx_buffer_data) {
        free(inst->tx_buffer_data);
        inst->tx_buffer_data = NULL;
    }
    
    inst->initialized = false;
    
    return ERROR_CODE_SUCCESS;
}

// TX interrupt handlers
ISR(USART1_UDRE_vect) {
    uart_instance_t* inst = &uart_instances[UART_PORT_1];
    uint8_t data;
    
    if (ring_buffer_pop(&inst->tx_buffer, &data) == ERROR_CODE_SUCCESS) {
        *uart_registers[UART_PORT_1].UDRn = data;
    } else {
        // No more data to send, disable TX interrupt
        *uart_registers[UART_PORT_1].UCSRnB &= ~uart_registers[UART_PORT_1].UDRIEn;
    }
}

ISR(USART2_UDRE_vect) {
    uart_instance_t* inst = &uart_instances[UART_PORT_2];
    uint8_t data;
    
    if (ring_buffer_pop(&inst->tx_buffer, &data) == ERROR_CODE_SUCCESS) {
        *uart_registers[UART_PORT_2].UDRn = data;
    } else {
        // No more data to send, disable TX interrupt
        *uart_registers[UART_PORT_2].UCSRnB &= ~uart_registers[UART_PORT_2].UDRIEn;
    }
}

ISR(USART3_UDRE_vect) {
    uart_instance_t* inst = &uart_instances[UART_PORT_3];
    uint8_t data;
    
    if (ring_buffer_pop(&inst->tx_buffer, &data) == ERROR_CODE_SUCCESS) {
        *uart_registers[UART_PORT_3].UDRn = data;
    } else {
        // No more data to send, disable TX interrupt
        *uart_registers[UART_PORT_3].UCSRnB &= ~uart_registers[UART_PORT_3].UDRIEn;
    }
}

// UART port control blocks
static struct {
    uart_config_t config;
    uart_status_t status;
    ring_buffer_t rx_buffer;
    ring_buffer_t tx_buffer;
    uint8_t* rx_dma_buffer;
    uint8_t* tx_dma_buffer;
    bool dma_tx_active;
    bool dma_rx_active;
    uint8_t dma_tx_channel;
    uint8_t dma_rx_channel;
} uart_ports[UART_PORT_MAX];

// DMA completion callbacks
static void uart_dma_tx_complete(void) {
    for (uart_port_t port = 0; port < UART_PORT_MAX; port++) {
        if (uart_ports[port].dma_tx_active) {
            // Update status
            uart_instance_t* inst = &uart_instances[port];
            inst->status.tx_count += uart_ports[port].config.tx_buffer_size;
            
            // Call completion callback if registered
            if (uart_ports[port].config.tx_callback) {
                uart_ports[port].config.tx_callback(uart_ports[port].tx_dma_buffer,
                                                  uart_ports[port].config.tx_buffer_size);
            }
            
            uart_ports[port].dma_tx_active = false;
        }
    }
}

static void uart_dma_rx_complete(void) {
    for (uart_port_t port = 0; port < UART_PORT_MAX; port++) {
        if (uart_ports[port].dma_rx_active) {
            // Update status
            uart_instance_t* inst = &uart_instances[port];
            inst->status.rx_count += uart_ports[port].config.rx_buffer_size;
            
            // Copy data from DMA buffer to ring buffer
            for (uint16_t i = 0; i < uart_ports[port].config.rx_buffer_size; i++) {
                if (ring_buffer_push(&uart_ports[port].rx_buffer, 
                                   uart_ports[port].rx_dma_buffer[i]) != ERROR_CODE_SUCCESS) {
                    uart_update_status(port, UART_ERR_OVERFLOW);
                    break;
                }
            }
            
            // Call completion callback if registered
            if (uart_ports[port].config.rx_callback) {
                uart_ports[port].config.rx_callback(uart_ports[port].rx_dma_buffer,
                                                  uart_ports[port].config.rx_buffer_size);
            }
            
            // Restart DMA transfer for continuous reception if in circular mode
            if (uart_ports[port].config.dma_flags & UART_FLAG_DMA_CIRCULAR) {
                dma_start_transfer(uart_ports[port].dma_rx_channel);
            } else {
                uart_ports[port].dma_rx_active = false;
            }
        }
    }
}

static void uart_dma_error_handler(uint8_t channel, dma_error_t error) {
    for (uart_port_t port = 0; port < UART_PORT_MAX; port++) {
        if (uart_ports[port].dma_rx_channel == channel || 
            uart_ports[port].dma_tx_channel == channel) {
            
            // Update error statistics
            uart_ports[port].status.error_count++;
            
            // Map DMA error to UART error
            error_code_t uart_error;
            switch (error) {
                case DMA_ERR_OVERFLOW:
                    uart_error = UART_ERR_OVERFLOW;
                    break;
                case DMA_ERR_TIMEOUT:
                    uart_error = UART_ERR_TIMEOUT;
                    break;
                case DMA_ERR_TRANSFER:
                    uart_error = UART_ERR_TRANSFER;
                    break;
                default:
                    uart_error = UART_ERR_UNKNOWN;
            }
            
            uart_update_status(port, uart_error);
            
            // Stop DMA transfer
            dma_stop_transfer(channel);
            uart_ports[port].dma_rx_active = false;
            uart_ports[port].dma_tx_active = false;
            
            // Attempt recovery
            if (uart_ports[port].status.error_count < UART_MAX_ERRORS) {
                // Restart DMA transfer
                if (channel == uart_ports[port].dma_rx_channel) {
                    uart_start_dma_rx(port);
                } else {
                    uart_start_dma_tx(port);
                }
            } else {
                // Too many errors, disable DMA
                uart_ports[port].config.dma_flags &= ~(UART_FLAG_DMA_RX | UART_FLAG_DMA_TX);
                error_record(ERR_UART_DMA_DISABLED);
            }
            
            break;
        }
    }
}

static void uart_start_dma_rx(uart_port_t port) {
    if (!(uart_ports[port].config.dma_flags & UART_FLAG_DMA_RX) || 
        uart_ports[port].dma_rx_active) {
        return;
    }
    
    // Configure DMA for reception
    dma_config_t dma_config = {
        .channel = uart_ports[port].dma_rx_channel,
        .buffer_size = uart_ports[port].config.rx_buffer_size,
        .source = (void*)&UDR0 + port * 0x10,
        .destination = uart_ports[port].rx_dma_buffer,
        .direction = DMA_DIR_PERIPH_TO_MEM,
        .mode = (uart_ports[port].config.dma_flags & UART_FLAG_DMA_CIRCULAR) ? 
                DMA_MODE_CIRCULAR : DMA_MODE_NORMAL,
        .priority = uart_ports[port].config.dma_priority,
        .error_callback = uart_dma_error_handler
    };
    
    if (dma_configure_channel(dma_config.channel, &dma_config) != ERROR_CODE_SUCCESS ||
        dma_start_transfer(dma_config.channel) != ERROR_CODE_SUCCESS) {
        uart_update_status(port, UART_ERR_DMA_CONFIG);
        return;
    }
    
    uart_ports[port].dma_rx_active = true;
}

static void uart_start_dma_tx(uart_port_t port) {
    if (!(uart_ports[port].config.dma_flags & UART_FLAG_DMA_TX) || 
        uart_ports[port].dma_tx_active) {
        return;
    }
    
    // Configure DMA for transmission
    dma_config_t dma_config = {
        .channel = uart_ports[port].dma_tx_channel,
        .buffer_size = uart_ports[port].config.tx_buffer_size,
        .source = uart_ports[port].tx_dma_buffer,
        .destination = (void*)&UDR0 + port * 0x10,
        .direction = DMA_DIR_MEM_TO_PERIPH,
        .mode = DMA_MODE_NORMAL,
        .priority = uart_ports[port].config.dma_priority,
        .error_callback = uart_dma_error_handler
    };
    
    if (dma_configure_channel(dma_config.channel, &dma_config) != ERROR_CODE_SUCCESS ||
        dma_start_transfer(dma_config.channel) != ERROR_CODE_SUCCESS) {
        uart_update_status(port, UART_ERR_DMA_CONFIG);
        return;
    }
    
    uart_ports[port].dma_tx_active = true;
}

error_code_t uart_init(uart_port_t port, const uart_config_t* config) {
    if (port >= UART_PORT_MAX || !config) {
        return ERR_INVALID_PARAM;
    }
    
    error_code_t err;
    
    // Store configuration
    uart_ports[port].config = *config;
    
    // Initialize ring buffers
    err = ring_buffer_init(&uart_ports[port].rx_buffer, config->rx_buffer_size);
    if (err != ERROR_CODE_SUCCESS) {
        return err;
    }
    
    err = ring_buffer_init(&uart_ports[port].tx_buffer, config->tx_buffer_size);
    if (err != ERROR_CODE_SUCCESS) {
        ring_buffer_deinit(&uart_ports[port].rx_buffer);
        return err;
    }
    
    // Initialize DMA if enabled
    if (config->dma_flags & (UART_FLAG_DMA_TX | UART_FLAG_DMA_RX)) {
        // Initialize DMA controller if not already done
        static bool dma_initialized = false;
        if (!dma_initialized) {
            err = dma_init();
            if (err != ERROR_CODE_SUCCESS) {
                return err;
            }
            dma_initialized = true;
        }
        
        // Allocate DMA buffers
        if (config->dma_flags & UART_FLAG_DMA_TX) {
            uart_ports[port].tx_dma_buffer = malloc(config->tx_buffer_size);
            if (!uart_ports[port].tx_dma_buffer) {
                return ERR_OUT_OF_MEMORY;
            }
            
            // Configure TX DMA channel
            dma_config_t dma_config = {
                .channel = DMA_UART_TX_CHANNEL + port * 2,
                .mode = (config->dma_flags & UART_FLAG_DMA_CIRCULAR) ? 
                        DMA_MODE_CIRCULAR : DMA_MODE_SINGLE,
                .priority = config->dma_priority,
                .buffer_size = config->tx_buffer_size,
                .source = uart_ports[port].tx_dma_buffer,
                .destination = (void*)&UDR0 + port * 0x10, // UART data register
                .circular_mode = (config->dma_flags & UART_FLAG_DMA_CIRCULAR) > 0,
                .callback = uart_dma_tx_complete
            };
            
            err = dma_configure_channel(dma_config.channel, &dma_config);
            if (err != ERROR_CODE_SUCCESS) {
                free(uart_ports[port].tx_dma_buffer);
                return err;
            }
            
            uart_ports[port].dma_tx_channel = dma_config.channel;
        }
        
        if (config->dma_flags & UART_FLAG_DMA_RX) {
            uart_ports[port].rx_dma_buffer = malloc(config->rx_buffer_size);
            if (!uart_ports[port].rx_dma_buffer) {
                if (uart_ports[port].tx_dma_buffer) {
                    free(uart_ports[port].tx_dma_buffer);
                }
                return ERR_OUT_OF_MEMORY;
            }
            
            // Configure RX DMA channel
            dma_config_t dma_config = {
                .channel = DMA_UART_RX_CHANNEL + port * 2,
                .mode = (config->dma_flags & UART_FLAG_DMA_CIRCULAR) ? 
                        DMA_MODE_CIRCULAR : DMA_MODE_SINGLE,
                .priority = config->dma_priority,
                .buffer_size = config->rx_buffer_size,
                .source = (void*)&UDR0 + port * 0x10, // UART data register
                .destination = uart_ports[port].rx_dma_buffer,
                .circular_mode = (config->dma_flags & UART_FLAG_DMA_CIRCULAR) > 0,
                .callback = uart_dma_rx_complete
            };
            
            err = dma_configure_channel(dma_config.channel, &dma_config);
            if (err != ERROR_CODE_SUCCESS) {
                free(uart_ports[port].rx_dma_buffer);
                if (uart_ports[port].tx_dma_buffer) {
                    free(uart_ports[port].tx_dma_buffer);
                }
                return err;
            }
            
            uart_ports[port].dma_rx_channel = dma_config.channel;
            
            // Start RX DMA immediately for continuous reception
            dma_start_transfer(dma_config.channel);
            uart_ports[port].dma_rx_active = true;
        }
    }
    
    return ERROR_CODE_SUCCESS;
}

error_code_t uart_write(uart_port_t port, const uint8_t* data, uint16_t size) {
    if (port >= UART_PORT_MAX || !data || size == 0) {
        return ERR_INVALID_PARAM;
    }
    
    // If DMA TX is enabled and not busy
    if ((uart_ports[port].config.dma_flags & UART_FLAG_DMA_TX) && 
        !uart_ports[port].dma_tx_active) {
        
        // Copy data to DMA buffer
        memcpy(uart_ports[port].tx_dma_buffer, data, size);
        
        // Configure and start DMA transfer
        dma_config_t dma_config = {
            .channel = uart_ports[port].dma_tx_channel,
            .buffer_size = size,
            .source = uart_ports[port].tx_dma_buffer,
            .destination = (void*)&UDR0 + port * 0x10
        };
        
        error_code_t err = dma_configure_channel(dma_config.channel, &dma_config);
        if (err != ERROR_CODE_SUCCESS) {
            return err;
        }
        
        err = dma_start_transfer(dma_config.channel);
        if (err != ERROR_CODE_SUCCESS) {
            return err;
        }
        
        uart_ports[port].dma_tx_active = true;
        return ERROR_CODE_SUCCESS;
    }
    
    // Fall back to normal transfer if DMA is not available
    uart_instance_t* inst = &uart_instances[port];
    error_code_t err = ERROR_CODE_SUCCESS;
    
    for (uint16_t i = 0; i < size; i++) {
        err = ring_buffer_push(&inst->tx_buffer, data[i]);
        if (err != ERROR_CODE_SUCCESS) {
            uart_update_status(port, err);
            return err;
        }
        inst->status.tx_count++;
    }
    
    // Enable TX interrupt to start transmission
    const uart_registers_t* regs = &uart_registers[port];
    *regs->UCSRnB |= regs->UDRIEn;
    
    return ERROR_CODE_SUCCESS;
}

/**
	uart initialization	

	BAUD_9600, BAUD_115200
*/
int  uart_init(uint32 baud)
{
    uart_config_t config;
    config.baud_rate = baud;
    config.rx_buffer_size = UART_BUFF_SIZE;
    config.tx_buffer_size = UART_BUFF_SIZE;
    
    return uart_init(UART_PORT_0, &config);
}

int uart_getchar(uint8 *c) 
{
    DISABLE_RXC();

    if (_rx_len > 0)
    {
        *c = _rx_buff[_rx_front];

        _rx_front = (_rx_front+1) % UART_BUFF_SIZE;
        _rx_len--;

        //enable_intr
        ENABLE_RXC();

        return 1;
    }
    //enable_intr
    ENABLE_RXC();

    return 0;
} 

int uart_putchar(uint8 c) 
{ 
    // disable_intr
    DISABLE_TXC();
    DISABLE_UDRE();
    
    if (_tx_len < UART_BUFF_SIZE)
    {
        _tx_buff[_tx_tail] = c;
        _tx_tail = (_tx_tail+1) % UART_BUFF_SIZE;
        _tx_len ++;

        //enable_intr
        ENABLE_UDRE();

        return 1;
    }

    //enable_intr
    ENABLE_UDRE();

    return 0;
}
