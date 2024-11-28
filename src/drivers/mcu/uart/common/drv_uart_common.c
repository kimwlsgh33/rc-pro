#include "drv_uart_common.h"
#include <string.h>

void uart_buffer_init(uart_buffer_t *buffer, uint8_t *data, uint16_t size) {
    buffer->data = data;
    buffer->size = size;
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
}

bool uart_buffer_write(uart_buffer_t *buffer, uint8_t byte) {
    if (buffer->count >= buffer->size) {
        return false;
    }

    buffer->data[buffer->head] = byte;
    buffer->head = (buffer->head + 1) % buffer->size;
    buffer->count++;
    return true;
}

bool uart_buffer_read(uart_buffer_t *buffer, uint8_t *byte) {
    if (buffer->count == 0) {
        return false;
    }

    *byte = buffer->data[buffer->tail];
    buffer->tail = (buffer->tail + 1) % buffer->size;
    buffer->count--;
    return true;
}

uint16_t uart_buffer_available(const uart_buffer_t *buffer) {
    return buffer->count;
}

uint16_t uart_buffer_space(const uart_buffer_t *buffer) {
    return buffer->size - buffer->count;
}

void uart_buffer_flush(uart_buffer_t *buffer) {
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
}

uint16_t uart_calculate_brr(uint32_t baud_rate, uint32_t cpu_freq, bool u2x) {
    uint16_t brr;
    
    if (u2x) {
        brr = (cpu_freq / (8UL * baud_rate)) - 1;
    } else {
        brr = (cpu_freq / (16UL * baud_rate)) - 1;
    }
    
    return brr;
}
