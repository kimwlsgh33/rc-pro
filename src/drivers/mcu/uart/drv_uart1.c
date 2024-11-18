#include "drv_uart1.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Buffer for UART1
static uint8_t _rx_buff[UART1_BUFFER_SIZE];
static uint8_t _rx_front = 0;
static uint8_t _rx_tail = 0;
static uint8_t _rx_len = 0;

static uint8_t _tx_buff[UART1_BUFFER_SIZE];
static uint8_t _tx_front = 0;
static uint8_t _tx_tail = 0;
static uint8_t _tx_len = 0;

// UART1 status structure
typedef struct {
    uint16_t error_count;
    uint16_t framing_errors;
    uint16_t overflow_count;
    uint16_t parity_errors;
    uint16_t rx_count;
    uint16_t tx_count;
    uint8_t last_error;
} uart1_status_t;

// UART1 status
uart1_status_t uart1_status;

// UART1 error callback
void (*uart1_error_callback)(uint8_t error);

// UART1 RX callback
void (*uart1_rx_callback)(uint8_t data);

// UART1 TX callback
void (*uart1_tx_callback)(void);

ISR(USART1_RX_vect) {
    uint8_t status = UCSR1A;
    uint8_t data = UDR1;
    bool buffer_full = false;
    
    // Check for hardware errors first
    if (status & ((1 << FE1) | (1 << DOR1) | (1 << UPE1))) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            uart1_status.error_count++;
            
            if (status & (1 << FE1)) {
                uart1_status.framing_errors++;
                uart1_status.last_error = UART_ERR_FRAMING;
            }
            if (status & (1 << DOR1)) {
                uart1_status.overflow_count++;
                uart1_status.last_error = UART_ERR_OVERFLOW;
            }
            if (status & (1 << UPE1)) {
                uart1_status.parity_errors++;
                uart1_status.last_error = UART_ERR_PARITY;
            }
        }
        
        // Call error callback if registered
        if (uart1_error_callback) {
            uart1_error_callback(uart1_status.last_error);
        }
        return;
    }
    
    // Check for buffer overflow before storing
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_rx_len >= UART1_BUFFER_SIZE) {
            buffer_full = true;
            uart1_status.overflow_count++;
            uart1_status.last_error = UART_ERR_OVERFLOW;
        }
    }
    
    if (buffer_full) {
        if (uart1_error_callback) {
            uart1_error_callback(UART_ERR_OVERFLOW);
        }
        return;
    }
    
    // Store data and update statistics atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _rx_buff[_rx_tail] = data;
        _rx_tail = (_rx_tail + 1) % UART1_BUFFER_SIZE;
        _rx_len++;
        uart1_status.rx_count++;
        
        // Call RX callback if registered
        if (uart1_rx_callback) {
            uart1_rx_callback(data);
        }
    }
}

ISR(USART1_UDRE_vect) {
    bool has_data = false;
    uint8_t data = 0;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_tx_len > 0) {
            has_data = true;
            data = _tx_buff[_tx_front];
            _tx_front = (_tx_front + 1) % UART1_BUFFER_SIZE;
            _tx_len--;
            uart1_status.tx_count++;
        }
    }
    
    if (has_data) {
        UDR1 = data;
    } else {
        // Disable UDRE interrupt when no more data
        UCSR1B &= ~(1 << UDRIE1);
        
        // Call TX callback if registered
        if (uart1_tx_callback) {
            uart1_tx_callback();
        }
    }
}

error_code_t uart1_init(uint32_t baud) {
    // Validate baud rate
    if (baud == 0) {
        return ERR_INVALID_PARAM;
    }
    
    // Calculate baud rate register value and verify it's within acceptable range
    uint16_t baud_setting = (F_CPU / 8 / baud - 1);
    if (baud_setting > 0xFFF) {  // 12-bit max for baud rate generator
        return ERR_INVALID_PARAM;
    }
    
    // Configure UART1 with interrupts disabled
    cli();
    
    // Configure UART1 hardware
    UCSR1A = (1 << U2X1);  // Double speed mode
    UCSR1B = (1 << RXCIE1) |  // RX Complete Interrupt Enable
             (1 << TXCIE1) |  // TX Complete Interrupt Enable
             (1 << RXEN1)  |  // Receiver Enable
             (1 << TXEN1);    // Transmitter Enable
    UCSR1C = (3 << UCSZ10);  // 8-bit data, 1 stop bit, no parity
    
    // Set baud rate
    UBRR1H = baud_setting >> 8;
    UBRR1L = baud_setting;
    
    // Verify configuration was applied correctly
    if (UCSR1A != (1 << U2X1) ||
        UCSR1B != ((1 << RXCIE1) | (1 << TXCIE1) | (1 << RXEN1) | (1 << TXEN1)) ||
        UCSR1C != (3 << UCSZ10) ||
        UBRR1H != (baud_setting >> 8) ||
        UBRR1L != (baud_setting & 0xFF)) {
        sei();
        return ERR_HARDWARE;
    }
    
    // Initialize buffers atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _rx_front = _rx_tail = _rx_len = 0;
        _tx_front = _tx_tail = _tx_len = 0;
    }
    
    // Initialize status structure
    memset(&uart1_status, 0, sizeof(uart1_status_t));
    
    // Clear callbacks
    uart1_error_callback = NULL;
    uart1_rx_callback = NULL;
    uart1_tx_callback = NULL;
    
    sei();
    return ERROR_CODE_SUCCESS;
}

error_code_t uart1_deinit(void) {
    // Disable all UART1 interrupts and functionality
    UCSR1B = 0;
    
    // Clear status and variables atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Clear status
        memset(&uart1_status, 0, sizeof(uart1_status_t));
        
        // Reset buffer state
        _rx_front = _rx_tail = _rx_len = 0;
        _tx_front = _tx_tail = _tx_len = 0;
        
        // Clear callbacks
        uart1_error_callback = NULL;
        uart1_rx_callback = NULL;
        uart1_tx_callback = NULL;
    }
    
    return ERROR_CODE_SUCCESS;
}

int uart1_getchar(uint8_t *c) {
    bool success = false;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_rx_len > 0) {
            *c = _rx_buff[_rx_front];
            _rx_front = (_rx_front + 1) % UART1_BUFFER_SIZE;
            _rx_len--;
            success = true;
        }
    }
    
    return success ? 1 : 0;
}

int uart1_putchar(uint8_t c) {
    bool success = false;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_tx_len < UART1_BUFFER_SIZE) {
            _tx_buff[_tx_tail] = c;
            _tx_tail = (_tx_tail + 1) % UART1_BUFFER_SIZE;
            _tx_len++;
            success = true;
        }
    }
    
    if (success) {
        // Enable UDRE interrupt
        UCSR1B |= (1 << UDRIE1);
        return 1;
    }
    
    return 0;
}

void uart1_set_callbacks(void (*error_cb)(uint8_t), 
                        void (*rx_cb)(uint8_t),
                        void (*tx_cb)(void)) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        uart1_error_callback = error_cb;
        uart1_rx_callback = rx_cb;
        uart1_tx_callback = tx_cb;
    }
}

bool uart1_is_rx_ready(void) {
    bool ready;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ready = (_rx_len > 0);
    }
    return ready;
}

bool uart1_is_tx_ready(void) {
    bool ready;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ready = (_tx_len < UART1_BUFFER_SIZE);
    }
    return ready;
}

const uart1_status_t* uart1_get_status(void) {
    return &uart1_status;
}
