#include "drv_uart2.h"
#include "drv_uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

// Static driver instance
static uart_driver_t uart2_driver;
static uart_status_t uart2_status;
static uart_config_t uart2_config;

// Buffer management
static uint8_t _rx_buff[UART2_BUFFER_SIZE];
static uint8_t _tx_buff[UART2_BUFFER_SIZE];
static volatile struct {
    uint8_t front;
    uint8_t tail;
    uint8_t len;
} _rx_buf = {0}, _tx_buf = {0};

// Callback functions
void (*uart2_error_callback)(uint8_t error);
void (*uart2_rx_callback)(uint8_t data);
void (*uart2_tx_callback)(void);

// Forward declarations
static error_code_t uart2_write(const uint8_t* data, uint16_t size);
static error_code_t uart2_read(uint8_t* data, uint16_t size, uint16_t* bytes_read);
static error_code_t uart2_write_byte(uint8_t byte);
static error_code_t uart2_read_byte(uint8_t* byte);
static error_code_t uart2_flush_rx(void);
static error_code_t uart2_flush_tx(void);
static uint16_t uart2_rx_available(void);
static uint16_t uart2_tx_space(void);
static const uart_status_t* uart2_get_status(void);

// Driver interface implementation
static error_code_t uart2_driver_init(void) {
    return uart2_init(&uart2_config);
}

static error_code_t uart2_driver_deinit(void) {
    return uart2_deinit();
}

static error_code_t uart2_driver_configure(const driver_config_t* config) {
    if (!config) return ERR_INVALID_PARAM;
    memcpy(&uart2_config.driver, config, sizeof(driver_config_t));
    return ERR_SUCCESS;
}

static error_code_t uart2_driver_get_status(driver_status_t* status) {
    if (!status) return ERR_INVALID_PARAM;
    status->state = uart2_status.state;
    status->error = uart2_status.last_error;
    return ERR_SUCCESS;
}

// Initialize UART2 driver interface
static void uart2_init_driver(void) {
    // Setup driver interface
    uart2_driver.driver.init = uart2_driver_init;
    uart2_driver.driver.deinit = uart2_driver_deinit;
    uart2_driver.driver.configure = uart2_driver_configure;
    uart2_driver.driver.get_status = uart2_driver_get_status;
    
    // Setup UART specific functions
    uart2_driver.write = uart2_write;
    uart2_driver.read = uart2_read;
    uart2_driver.write_byte = uart2_write_byte;
    uart2_driver.read_byte = uart2_read_byte;
    uart2_driver.flush_rx = uart2_flush_rx;
    uart2_driver.flush_tx = uart2_flush_tx;
    uart2_driver.rx_available = uart2_rx_available;
    uart2_driver.tx_space = uart2_tx_space;
    uart2_driver.get_status = uart2_get_status;
}

error_code_t uart2_init(const uart_config_t* config) {
    if (!config) return ERR_INVALID_PARAM;
    
    // Validate configuration parameters
    if (config->baud_rate == 0 || 
        config->data_bits < 5 || config->data_bits > 9 ||
        config->stop_bits < 1 || config->stop_bits > 2) {
        return ERR_INVALID_PARAM;
    }
    
    // Calculate and validate baud rate register value
    uint16_t baud_setting = (F_CPU / 8 / config->baud_rate - 1);
    if (baud_setting > 0xFFF || baud_setting < 0x10) {  // Check both overflow and underflow
        return ERR_INVALID_PARAM;
    }
    
    // Disable interrupts during initialization
    cli();
    
    // Configure UART2 hardware
    UCSR2A = (1 << U2X2);  // Double speed mode
    UCSR2B = (1 << RXCIE2) |  // RX Complete Interrupt Enable
             (1 << TXCIE2) |  // TX Complete Interrupt Enable
             (1 << RXEN2)  |  // Receiver Enable
             (1 << TXEN2);    // Transmitter Enable
    UCSR2C = (3 << UCSZ20);  // 8-bit data, 1 stop bit, no parity
    
    // Set baud rate
    UBRR2H = baud_setting >> 8;
    UBRR2L = baud_setting;
    
    // Verify configuration
    if (UCSR2A != (1 << U2X2) ||
        UCSR2B != ((1 << RXCIE2) | (1 << TXCIE2) | (1 << RXEN2) | (1 << TXEN2)) ||
        UCSR2C != (3 << UCSZ20) ||
        UBRR2H != (baud_setting >> 8) ||
        UBRR2L != (baud_setting & 0xFF)) {
        sei();
        return ERR_HARDWARE;
    }
    
    // Initialize buffers atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _rx_buf.front = _rx_buf.tail = _rx_buf.len = 0;
        _tx_buf.front = _tx_buf.tail = _tx_buf.len = 0;
        
        // Initialize status with complete reset
        memset(&uart2_status, 0, sizeof(uart_status_t));
        uart2_status.state = DRIVER_STATE_INITIALIZED;
        uart2_status.error_count = 0;
        uart2_status.overflow_count = 0;
        uart2_status.framing_errors = 0;
        uart2_status.parity_errors = 0;
        uart2_status.last_error = UART_ERR_NONE;
        
        // Clear callbacks
        uart2_error_callback = NULL;
        uart2_rx_callback = NULL;
        uart2_tx_callback = NULL;
    }
    
    sei();
    return ERR_SUCCESS;
}

void uart2_set_callbacks(void (*error_cb)(uint8_t),
                        void (*rx_cb)(uint8_t),
                        void (*tx_cb)(void)) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        uart2_error_callback = error_cb;
        uart2_rx_callback = rx_cb;
        uart2_tx_callback = tx_cb;
    }
}

error_code_t uart2_deinit(void) {
    // Disable all UART2 interrupts and functionality
    UCSR2B = 0;
    
    // Clear status and variables atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Clear status
        memset(&uart2_status, 0, sizeof(uart_status_t));
        
        // Reset buffer state
        _rx_buf.front = _rx_buf.tail = _rx_buf.len = 0;
        _tx_buf.front = _tx_buf.tail = _tx_buf.len = 0;
        
        // Clear configuration
        memset(&uart2_config, 0, sizeof(uart_config_t));
    }
    
    return ERR_SUCCESS;
}

// Interrupt handlers
ISR(USART2_RX_vect) {
    uint8_t status = UCSR2A;
    uint8_t data = UDR2;
    bool should_call_error_cb = false;
    bool should_call_rx_cb = false;
    uint8_t error_type = UART_ERR_NONE;
    
    // Check for hardware errors first
    if (status & ((1 << FE2) | (1 << DOR2) | (1 << UPE2))) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            uart2_status.error_count++;
            
            if (status & (1 << FE2)) {
                uart2_status.framing_errors++;
                uart2_status.last_error = UART_ERR_FRAMING;
                error_type = UART_ERR_FRAMING;
            }
            if (status & (1 << DOR2)) {
                uart2_status.overflow_count++;
                uart2_status.last_error = UART_ERR_OVERFLOW;
                error_type = UART_ERR_OVERFLOW;
            }
            if (status & (1 << UPE2)) {
                uart2_status.parity_errors++;
                uart2_status.last_error = UART_ERR_PARITY;
                error_type = UART_ERR_PARITY;
            }
            
            should_call_error_cb = (uart2_error_callback != NULL);
        }
        
        // Call error callback outside ATOMIC_BLOCK
        if (should_call_error_cb) {
            uart2_error_callback(error_type);
        }
        return;
    }
    
    // Check for buffer overflow before writing
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_rx_buf.len >= UART2_BUFFER_SIZE) {
            uart2_status.overflow_count++;
            uart2_status.last_error = UART_ERR_OVERFLOW;
            should_call_error_cb = (uart2_error_callback != NULL);
            error_type = UART_ERR_OVERFLOW;
        } else {
            // Store data in buffer
            _rx_buff[_rx_buf.tail] = data;
            _rx_buf.tail = (_rx_buf.tail + 1) % UART2_BUFFER_SIZE;
            _rx_buf.len++;
            
            should_call_rx_cb = (uart2_rx_callback != NULL);
        }
    }
    
    // Call callbacks outside ATOMIC_BLOCK
    if (should_call_error_cb) {
        uart2_error_callback(error_type);
    }
    if (should_call_rx_cb) {
        uart2_rx_callback(data);
    }
}

ISR(USART2_UDRE_vect) {
    bool has_data = false;
    uint8_t data = 0;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_tx_buf.len > 0) {
            has_data = true;
            data = _tx_buff[_tx_buf.front];
            _tx_buf.front = (_tx_buf.front + 1) % UART2_BUFFER_SIZE;
            _tx_buf.len--;
            uart2_status.tx_count++;
        }
    }
    
    if (has_data) {
        UDR2 = data;
    } else {
        // Disable UDRE interrupt when no more data
        UCSR2B &= ~(1 << UDRIE2);
        
        // Call TX callback if registered
        if (uart2_tx_callback) {
            uart2_tx_callback();
        }
    }
}

// Implementation of driver interface functions
static error_code_t uart2_write(const uint8_t* data, uint16_t size) {
    if (!data || size == 0) return ERR_INVALID_PARAM;
    
    for (uint16_t i = 0; i < size; i++) {
        if (uart2_write_byte(data[i]) != ERR_SUCCESS) {
            return ERR_UART_OVERFLOW;
        }
    }
    return ERR_SUCCESS;
}

static error_code_t uart2_read(uint8_t* data, uint16_t size, uint16_t* bytes_read) {
    if (!data || !bytes_read) return ERR_INVALID_PARAM;
    
    *bytes_read = 0;
    while (*bytes_read < size && uart2_rx_available()) {
        if (uart2_read_byte(&data[*bytes_read]) == ERR_SUCCESS) {
            (*bytes_read)++;
        }
    }
    return ERR_SUCCESS;
}

static error_code_t uart2_write_byte(uint8_t byte) {
    bool success = false;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_tx_buf.len < UART2_BUFFER_SIZE) {
            _tx_buff[_tx_buf.tail] = byte;
            _tx_buf.tail = (_tx_buf.tail + 1) % UART2_BUFFER_SIZE;
            _tx_buf.len++;
            success = true;
        }
    }
    
    if (success) {
        // Enable UDRE interrupt
        UCSR2B |= (1 << UDRIE2);
        return ERR_SUCCESS;
    }
    
    return ERR_UART_OVERFLOW;
}

static error_code_t uart2_read_byte(uint8_t* byte) {
    if (!byte) return ERR_INVALID_PARAM;
    
    bool success = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_rx_buf.len > 0) {
            *byte = _rx_buff[_rx_buf.front];
            _rx_buf.front = (_rx_buf.front + 1) % UART2_BUFFER_SIZE;
            _rx_buf.len--;
            success = true;
        }
    }
    
    return success ? ERR_SUCCESS : ERR_NOT_READY;
}

static error_code_t uart2_flush_rx(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _rx_buf.front = _rx_buf.tail = _rx_buf.len = 0;
    }
    return ERR_SUCCESS;
}

static error_code_t uart2_flush_tx(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _tx_buf.front = _tx_buf.tail = _tx_buf.len = 0;
        UCSR2B &= ~(1 << UDRIE2);  // Disable UDRE interrupt
    }
    return ERR_SUCCESS;
}

static uint16_t uart2_rx_available(void) {
    uint16_t len;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        len = _rx_buf.len;
    }
    return len;
}

static uint16_t uart2_tx_space(void) {
    uint16_t space;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        space = UART2_BUFFER_SIZE - _tx_buf.len;
    }
    return space;
}

static const uart_status_t* uart2_get_status(void) {
    return &uart2_status;
}

// Public function to get driver interface
const uart_driver_t* uart2_get_driver(void) {
    return &uart2_driver;
}
