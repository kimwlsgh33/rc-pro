// System includes
#include <string.h>

// AVR includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Project includes
#include "../../../../config/cfg_build.h"
#include "drv_uart2.h"
#include "../common/drv_uart_types.h"

// Forward declarations of static functions
static error_code_t uart2_driver_init(void);
static error_code_t uart2_driver_deinit(void);
static error_code_t uart2_driver_configure(const driver_config_t *config);
static error_code_t uart2_driver_get_status(driver_status_t *status);
static void uart2_init_driver(void);
static error_code_t uart2_write(const uint8_t *data, uint16_t size);
static error_code_t uart2_read(uint8_t *data, uint16_t size, uint16_t *bytes_read);
static error_code_t uart2_write_byte(uint8_t byte);
static error_code_t uart2_read_byte(uint8_t *byte);
static error_code_t uart2_flush_rx(void);
static error_code_t uart2_flush_tx(void);
static uint16_t uart2_rx_available(void);
static uint16_t uart2_tx_space(void);
static uart_status_t uart2_get_status(void);

// Static driver instance
static uart_driver_t uart2_driver;
static uart_status_t uart2_status;
static uart_config_t uart2_config;

// Buffer management
static uint8_t _rx_buff[UART2_BUFFER_SIZE];
static uint8_t _tx_buff[UART2_BUFFER_SIZE];
static uart_buffer_t _rx_buf = {
    .data = _rx_buff,
    .size = UART2_BUFFER_SIZE,
    .head = 0,
    .tail = 0,
    .count = 0
};
static uart_buffer_t _tx_buf = {
    .data = _tx_buff,
    .size = UART2_BUFFER_SIZE,
    .head = 0,
    .tail = 0,
    .count = 0
};

// Callback functions
void (*uart2_error_callback)(uint8_t error);
void (*uart2_rx_callback)(uint8_t data);
void (*uart2_tx_callback)(void);

// Buffer helper functions
static inline bool buffer_is_full(const uart_buffer_t *buf) {
    return buf->count >= buf->size;
}

static inline bool buffer_is_empty(const uart_buffer_t *buf) {
    return buf->count == 0;
}

static inline uint16_t buffer_available_space(const uart_buffer_t *buf) {
    return buf->size - buf->count;
}

static inline error_code_t buffer_write(uart_buffer_t *buf, uint8_t byte) {
    if (buffer_is_full(buf)) {
        return ERR_UART_OVERFLOW;
    }
    
    buf->data[buf->tail] = byte;
    buf->tail = (buf->tail + 1) % buf->size;
    buf->count++;
    return ERR_SUCCESS;
}

static inline error_code_t buffer_read(uart_buffer_t *buf, uint8_t *byte) {
    if (buffer_is_empty(buf)) {
        return ERR_UART_NO_DATA;
    }
    
    *byte = buf->data[buf->head];
    buf->head = (buf->head + 1) % buf->size;
    buf->count--;
    return ERR_SUCCESS;
}

// Driver interface implementation
static error_code_t uart2_driver_init(void) { return uart2_init(&uart2_config); }

static error_code_t uart2_driver_deinit(void) { return uart2_deinit(); }

static error_code_t uart2_driver_configure(const driver_config_t *config)
{
  if (!config)
    return ERR_INVALID_PARAM;
  memcpy(&uart2_config.driver, config, sizeof(driver_config_t));
  return ERR_SUCCESS;
}

static error_code_t uart2_driver_get_status(driver_status_t *status)
{
  if (!status)
    return ERR_INVALID_PARAM;
  
  // Map UART-specific status to generic driver status
  status->state = (uart2_status.error_count > 0) ? 
                  DRIVER_STATE_ERROR : DRIVER_STATE_CONFIGURED;
  status->error = (uart2_status.error_count > 0) ? 
                  ERR_INVALID_STATE : ERR_SUCCESS;
  status->status = &uart2_status;  // Optional: attach full UART status
  
  return ERR_SUCCESS;
}

// Initialize UART2 driver interface
static void uart2_init_driver(void)
{
  // Setup driver interface
  uart2_driver.driver.init       = uart2_driver_init;
  uart2_driver.driver.deinit     = uart2_driver_deinit;
  uart2_driver.driver.configure  = uart2_driver_configure;
  uart2_driver.driver.get_status = uart2_driver_get_status;

  // Setup UART specific functions
  uart2_driver.write        = uart2_write;
  uart2_driver.read         = uart2_read;
  uart2_driver.write_byte   = uart2_write_byte;
  uart2_driver.read_byte    = uart2_read_byte;
  uart2_driver.flush_rx     = uart2_flush_rx;
  uart2_driver.flush_tx     = uart2_flush_tx;
  uart2_driver.rx_available = uart2_rx_available;
  uart2_driver.tx_space     = uart2_tx_space;
  uart2_driver.get_status   = uart2_get_status;
}

error_code_t uart2_init(const uart_config_t *config)
{
  if (!config)
    return ERR_INVALID_PARAM;

  // Validate configuration parameters
  if (config->baud_rate == 0 || config->data_bits < 5 || config->data_bits > 9 ||
      config->stop_bits < 1 || config->stop_bits > 2) {
    return ERR_INVALID_PARAM;
  }

  // Calculate and validate baud rate register value
  uint16_t baud_setting = (F_CPU / 8 / config->baud_rate - 1);
  if (baud_setting > 0xFFF || baud_setting < 0x10) { // Check both overflow and underflow
    return ERR_INVALID_PARAM;
  }

  // Disable interrupts during initialization
  cli();

  // Configure UART2 hardware
  UCSR2A = (1 << U2X2);    // Double speed mode
  UCSR2B = (1 << RXCIE2) | // RX Complete Interrupt Enable
           (1 << TXCIE2) | // TX Complete Interrupt Enable
           (1 << RXEN2) |  // Receiver Enable
           (1 << TXEN2);   // Transmitter Enable
  UCSR2C = (3 << UCSZ20);  // 8-bit data, 1 stop bit, no parity

  // Set baud rate
  UBRR2H = baud_setting >> 8;
  UBRR2L = baud_setting;

  // Verify configuration
  if (UCSR2A != (1 << U2X2) ||
      UCSR2B != ((1 << RXCIE2) | (1 << TXCIE2) | (1 << RXEN2) | (1 << TXEN2)) ||
      UCSR2C != (3 << UCSZ20) || UBRR2H != (baud_setting >> 8) || UBRR2L != (baud_setting & 0xFF)) {
    sei();
    return ERR_UART_REGS;
  }

  // Initialize buffers atomically
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    _rx_buf.head = _rx_buf.tail = _rx_buf.count = 0;
    _tx_buf.head = _tx_buf.tail = _tx_buf.count = 0;

    // Initialize status with complete reset
    memset(&uart2_status, 0, sizeof(uart_status_t));
    uart2_status.state          = DRIVER_STATE_INITIALIZED;
    uart2_status.error_count    = 0;
    uart2_status.overflow_count = 0;
    uart2_status.framing_errors = 0;
    uart2_status.parity_errors  = 0;
    uart2_status.last_error     = 0;

    // Clear callbacks
    uart2_error_callback = NULL;
    uart2_rx_callback    = NULL;
    uart2_tx_callback    = NULL;
  }

  sei();
  return ERR_SUCCESS;
}

void uart2_set_callbacks(void (*error_cb)(uint8_t), void (*rx_cb)(uint8_t), void (*tx_cb)(void))
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    uart2_error_callback = error_cb;
    uart2_rx_callback    = rx_cb;
    uart2_tx_callback    = tx_cb;
  }
}

error_code_t uart2_deinit(void)
{
  // Disable all UART2 interrupts and functionality
  UCSR2B = 0;

  // Clear status and variables atomically
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    // Clear status
    memset(&uart2_status, 0, sizeof(uart_status_t));

    // Reset buffer state
    _rx_buf.head = _rx_buf.tail = _rx_buf.count = 0;
    _tx_buf.head = _tx_buf.tail = _tx_buf.count = 0;

    // Clear configuration
    memset(&uart2_config, 0, sizeof(uart_config_t));
  }

  return ERR_SUCCESS;
}

// Interrupt handlers
ISR(USART2_RX_vect)
{
  uint8_t status            = UCSR2A;
  uint8_t data              = UDR2;
  bool should_call_error_cb = false;
  bool should_call_rx_cb    = false;
  uint16_t error_type        = ERR_NOT_INIT;

  // Check for hardware errors first
  if (status & ((1 << FE2) | (1 << DOR2) | (1 << UPE2))) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      uart2_status.error_count++;

      if (status & (1 << FE2)) {
        uart2_status.framing_errors++;
        uart2_status.last_error = ERR_UART_FRAME;
        error_type              = ERR_UART_FRAME;
      }
      if (status & (1 << DOR2)) {
        uart2_status.overflow_count++;
        uart2_status.last_error = ERR_UART_OVERFLOW;
        error_type              = ERR_UART_OVERFLOW;
      }
      if (status & (1 << UPE2)) {
        uart2_status.parity_errors++;
        uart2_status.last_error = ERR_UART_PARITY;
        error_type              = ERR_UART_PARITY;
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
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if (buffer_is_full(&_rx_buf)) {
      uart2_status.overflow_count++;
      uart2_status.last_error = ERR_UART_OVERFLOW;
      should_call_error_cb    = (uart2_error_callback != NULL);
      error_type              = ERR_UART_OVERFLOW;
    } else {
      // Store data in buffer
      buffer_write(&_rx_buf, data);
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

ISR(USART2_UDRE_vect)
{
  bool has_data = false;
  uint8_t data  = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if (!buffer_is_empty(&_tx_buf)) {
      has_data      = true;
      buffer_read(&_tx_buf, &data);
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

// Internal helper functions
static error_code_t uart2_write(const uint8_t *data, uint16_t size)
{
  if (!data || size == 0)
    return ERR_INVALID_PARAM;

  for (uint16_t i = 0; i < size; i++) {
    if (uart2_write_byte(data[i]) != ERR_SUCCESS) {
      return ERR_UART_OVERFLOW;
    }
  }
  return ERR_SUCCESS;
}

static error_code_t uart2_read(uint8_t *data, uint16_t size, uint16_t *bytes_read)
{
  if (!data || !bytes_read)
    return ERR_INVALID_PARAM;

  *bytes_read = 0;
  while (*bytes_read < size && uart2_rx_available()) {
    if (uart2_read_byte(&data[*bytes_read]) == ERR_SUCCESS) {
      (*bytes_read)++;
    }
  }
  return ERR_SUCCESS;
}

static error_code_t uart2_write_byte(uint8_t byte)
{
  return buffer_write(&_tx_buf, byte);
}

static error_code_t uart2_read_byte(uint8_t *byte)
{
  return buffer_read(&_rx_buf, byte);
}

static error_code_t uart2_flush_rx(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    _rx_buf.head = _rx_buf.tail = _rx_buf.count = 0;
  }
  return ERR_SUCCESS;
}

static error_code_t uart2_flush_tx(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    _tx_buf.head = _tx_buf.tail = _tx_buf.count = 0;
    // Disable UDRE interrupt
    UCSR2B &= ~(1 << UDRIE2);
  }
  return ERR_SUCCESS;
}

static uint16_t uart2_rx_available(void)
{
    return _rx_buf.count;  // Return actual number of bytes available
}

static uint16_t uart2_tx_space(void)
{
  return buffer_available_space(&_tx_buf);
}

static uart_status_t uart2_get_status(void)
{
  return uart2_status;
}

// Public function to get driver interface
const uart_driver_t *uart2_get_driver(void) { return &uart2_driver; }
