#include "ring_buffer.h"
#include <string.h>
#include <avr/interrupt.h>

error_code_t ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t size) {
    if (!rb || !buffer || size == 0) {
        return ERR_INVALID_PARAM;
    }

    // Size must be power of 2 for efficient wrapping
    if ((size & (size - 1)) != 0) {
        return ERR_INVALID_PARAM;
    }

    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;

    // Clear buffer contents
    memset(buffer, 0, size);

    return ERROR_CODE_SUCCESS;
}

error_code_t ring_buffer_push(ring_buffer_t *rb, uint8_t data) {
    if (!rb || !rb->buffer) {
        return ERR_INVALID_PARAM;
    }

    uint8_t sreg = SREG;
    cli();  // Disable interrupts

    if (ring_buffer_is_full(rb)) {
        SREG = sreg;  // Restore interrupt state
        error_record(ERR_BUFFER_FULL);
        return ERR_BUFFER_FULL;
    }

    // Store data and update head pointer atomically
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) & (rb->size - 1);  // Efficient wrapping for power-of-2 sizes
    rb->count++;

    SREG = sreg;  // Restore interrupt state
    return ERROR_CODE_SUCCESS;
}

error_code_t ring_buffer_pop(ring_buffer_t *rb, uint8_t *data) {
    if (!rb || !rb->buffer || !data) {
        return ERR_INVALID_PARAM;
    }

    uint8_t sreg = SREG;
    cli();  // Disable interrupts

    if (ring_buffer_is_empty(rb)) {
        SREG = sreg;  // Restore interrupt state
        return ERR_BUFFER_EMPTY;
    }

    // Retrieve data and update tail pointer atomically
    *data = rb->buffer[rb->tail];
    rb->buffer[rb->tail] = 0;  // Clear data for security
    rb->tail = (rb->tail + 1) & (rb->size - 1);  // Efficient wrapping for power-of-2 sizes
    rb->count--;

    SREG = sreg;  // Restore interrupt state
    return ERROR_CODE_SUCCESS;
}

bool ring_buffer_is_empty(const ring_buffer_t *rb) {
    if (!rb) {
        return true;
    }
    return rb->count == 0;
}

bool ring_buffer_is_full(const ring_buffer_t *rb) {
    if (!rb) {
        return true;
    }
    return rb->count >= rb->size;
}

uint16_t ring_buffer_count(const ring_buffer_t *rb) {
    if (!rb) {
        return 0;
    }
    return rb->count;
}

void ring_buffer_clear(ring_buffer_t *rb) {
    if (!rb || !rb->buffer) {
        return;
    }

    uint8_t sreg = SREG;
    cli();  // Disable interrupts

    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    memset(rb->buffer, 0, rb->size);  // Clear buffer contents for security

    SREG = sreg;  // Restore interrupt state
}
