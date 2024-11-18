#include "drv_uart3.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Buffer for UART3 (ultrasonic sensor)
static uint8_t _rx_buff[UART3_BUFFER_SIZE];
static uint8_t _rx_front = 0;
static uint8_t _rx_tail = 0;
static uint8_t _rx_len = 0;

static uint8_t _tx_buff[UART3_BUFFER_SIZE];
static uint8_t _tx_front = 0;
static uint8_t _tx_tail = 0;
static uint8_t _tx_len = 0;

// Ultrasonic sensor specific variables
static bool _ultrasonic_mode = false;
static bool _data_ready = false;
static uint16_t _distance = 0;
static uint8_t _rx_state = 0;
static uint16_t _rx_data = 0;
static uint8_t _rx_sum = 0;

// UART3 status structure
typedef struct {
    uint16_t error_count;
    uint16_t framing_errors;
    uint16_t overflow_count;
    uint16_t parity_errors;
    uint16_t rx_count;
    uint16_t tx_count;
    uint8_t last_error;
} uart3_status_t;

uart3_status_t uart3_status;

// Callback function pointers
void (*uart3_error_callback)(uint8_t error);
void (*uart3_rx_callback)(uint8_t data);
void (*uart3_tx_callback)(void);

error_code_t uart3_init(uint32_t baud) {
    // Validate baud rate
    if (baud == 0) {
        return ERR_INVALID_PARAM;
    }
    
    // Calculate baud rate register value and verify it's within acceptable range
    uint16_t baud_setting = (F_CPU / 8 / baud - 1);
    if (baud_setting > 0xFFF) {  // 12-bit max for baud rate generator
        return ERR_INVALID_PARAM;
    }
    
    // Configure UART3 with interrupts disabled
    cli();
    
    // Configure UART3 hardware
    UCSR3A = (1 << U2X3);  // Double speed mode
    UCSR3B = (1 << RXCIE3) |  // RX Complete Interrupt Enable
             (1 << TXCIE3) |  // TX Complete Interrupt Enable
             (1 << RXEN3)  |  // Receiver Enable
             (1 << TXEN3);    // Transmitter Enable
    UCSR3C = (3 << UCSZ30);  // 8-bit data, 1 stop bit, no parity
    
    // Set baud rate
    UBRR3H = baud_setting >> 8;
    UBRR3L = baud_setting;
    
    // Verify configuration was applied correctly
    if (UCSR3A != (1 << U2X3) ||
        UCSR3B != ((1 << RXCIE3) | (1 << TXCIE3) | (1 << RXEN3) | (1 << TXEN3)) ||
        UCSR3C != (3 << UCSZ30) ||
        UBRR3H != (baud_setting >> 8) ||
        UBRR3L != (baud_setting & 0xFF)) {
        sei();
        return ERR_HARDWARE;
    }
    
    // Initialize buffers atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _rx_front = _rx_tail = _rx_len = 0;
        _tx_front = _tx_tail = _tx_len = 0;
        
        // Initialize ultrasonic variables
        _ultrasonic_mode = false;
        _data_ready = false;
        _distance = 0;
        _rx_state = 0;
        _rx_sum = 0;
        _rx_data = 0;
    }
    
    // Initialize status structure
    memset(&uart3_status, 0, sizeof(uart3_status_t));
    
    // Clear callbacks
    uart3_error_callback = NULL;
    uart3_rx_callback = NULL;
    uart3_tx_callback = NULL;
    
    sei();
    return ERROR_CODE_SUCCESS;
}

error_code_t uart3_deinit(void) {
    // Disable all UART3 interrupts and functionality
    UCSR3B = 0;
    
    // Clear status and variables atomically
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // Clear status
        memset(&uart3_status, 0, sizeof(uart3_status_t));
        
        // Reset buffer state
        _rx_front = _rx_tail = _rx_len = 0;
        _tx_front = _tx_tail = _tx_len = 0;
        
        // Reset ultrasonic mode
        _ultrasonic_mode = false;
        _data_ready = false;
        _distance = 0;
        _rx_state = 0;
        _rx_sum = 0;
        _rx_data = 0;
        
        // Clear callbacks
        uart3_error_callback = NULL;
        uart3_rx_callback = NULL;
        uart3_tx_callback = NULL;
    }
    
    return ERROR_CODE_SUCCESS;
}

void uart3_set_callbacks(void (*error_cb)(uint8_t), 
                        void (*rx_cb)(uint8_t),
                        void (*tx_cb)(void)) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        uart3_error_callback = error_cb;
        uart3_rx_callback = rx_cb;
        uart3_tx_callback = tx_cb;
    }
}

const uart3_status_t* uart3_get_status(void) {
    uart3_status_t* status = &uart3_status;
    return status;
}

bool uart3_is_rx_ready(void) {
    bool ready;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ready = (_rx_len > 0);
    }
    return ready;
}

bool uart3_is_tx_ready(void) {
    bool ready;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ready = (_tx_len < UART3_BUFFER_SIZE);
    }
    return ready;
}

bool uart3_is_ultrasonic_data_ready(void) {
    bool ready;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ready = _data_ready;
    }
    return ready;
}

uint16_t uart3_get_ultrasonic_distance(void) {
    uint16_t distance;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        distance = _distance;
        _data_ready = false;
    }
    return distance;
}

int uart3_putchar(uint8_t c) {
    bool success = false;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_tx_len < UART3_BUFFER_SIZE) {
            _tx_buff[_tx_tail] = c;
            _tx_tail = (_tx_tail + 1) % UART3_BUFFER_SIZE;
            _tx_len++;
            success = true;
        }
    }
    
    if (success) {
        // Enable UDRE interrupt
        UCSR3B |= (1 << UDRIE3);
        return 1;
    }
    
    return 0;
}

int uart3_getchar(uint8_t *c) {
    bool success = false;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_rx_len > 0) {
            *c = _rx_buff[_rx_front];
            _rx_front = (_rx_front + 1) % UART3_BUFFER_SIZE;
            _rx_len--;
            success = true;
        }
    }
    
    return success ? 1 : 0;
}

ISR(USART3_RX_vect) {
    uint8_t status = UCSR3A;
    uint8_t data = UDR3;
    bool buffer_full = false;
    
    // Check for hardware errors first
    if (status & ((1 << FE3) | (1 << DOR3) | (1 << UPE3))) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            uart3_status.error_count++;
            
            if (status & (1 << FE3)) {
                uart3_status.framing_errors++;
                uart3_status.last_error = UART_ERR_FRAMING;
            }
            if (status & (1 << DOR3)) {
                uart3_status.overflow_count++;
                uart3_status.last_error = UART_ERR_OVERFLOW;
            }
            if (status & (1 << UPE3)) {
                uart3_status.parity_errors++;
                uart3_status.last_error = UART_ERR_PARITY;
            }
        }
        
        // Call error callback if registered
        if (uart3_error_callback) {
            uart3_error_callback(uart3_status.last_error);
        }
        return;
    }
    
    if (_ultrasonic_mode) {
        // Process ultrasonic sensor protocol with error checking
        switch (_rx_state) {
            case 0:  // Wait for STX (0xFF)
                if (data == 0xFF) {
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                        _rx_state = 1;
                        _rx_sum = 0;
                        uart3_status.rx_count++;
                    }
                }
                break;
                
            case 1:  // High byte
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    _rx_data = data << 8;
                    _rx_sum += data;
                    _rx_state = 2;
                    uart3_status.rx_count++;
                }
                break;
                
            case 2:  // Low byte
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    _rx_data |= data;
                    _rx_sum += data;
                    _rx_state = 3;
                    uart3_status.rx_count++;
                }
                break;
                
            case 3:  // Checksum
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    if (_rx_sum == data) {
                        _distance = _rx_data;
                        _data_ready = true;
                        if (uart3_rx_callback) {
                            uart3_rx_callback(data);
                        }
                    } else {
                        uart3_status.error_count++;
                        uart3_status.last_error = UART_ERR_CHECKSUM;
                        if (uart3_error_callback) {
                            uart3_error_callback(UART_ERR_CHECKSUM);
                        }
                    }
                    _rx_state = 0;
                }
                break;
        }
    } else {
        // Check for buffer overflow before storing
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (_rx_len >= UART3_BUFFER_SIZE) {
                buffer_full = true;
                uart3_status.overflow_count++;
                uart3_status.last_error = UART_ERR_OVERFLOW;
            }
        }
        
        if (buffer_full) {
            if (uart3_error_callback) {
                uart3_error_callback(UART_ERR_OVERFLOW);
            }
            return;
        }
        
        // Store data and update statistics atomically
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            _rx_buff[_rx_tail] = data;
            _rx_tail = (_rx_tail + 1) % UART3_BUFFER_SIZE;
            _rx_len++;
            uart3_status.rx_count++;
            
            // Call RX callback if registered
            if (uart3_rx_callback) {
                uart3_rx_callback(data);
            }
        }
    }
}

ISR(USART3_UDRE_vect) {
    bool has_data = false;
    uint8_t data = 0;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_tx_len > 0) {
            has_data = true;
            data = _tx_buff[_tx_front];
            _tx_front = (_tx_front + 1) % UART3_BUFFER_SIZE;
            _tx_len--;
            uart3_status.tx_count++;
        }
    }
    
    if (has_data) {
        UDR3 = data;
    } else {
        // Disable UDRE interrupt when no more data
        UCSR3B &= ~(1 << UDRIE3);
        
        // Call TX callback if registered
        if (uart3_tx_callback) {
            uart3_tx_callback();
        }
    }
}

void uart3_set_ultrasonic_mode(bool enable) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _ultrasonic_mode = enable;
        _rx_state = 0;
        _data_ready = false;
        _rx_sum = 0;
        _rx_data = 0;
    }
}
