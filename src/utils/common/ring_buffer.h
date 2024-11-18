#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "../../utils/common/error_codes.h"

/**
 * @brief Ring buffer structure
 */
typedef struct {
    uint8_t *buffer;          // Pointer to buffer memory
    uint16_t size;            // Size of buffer
    volatile uint16_t head;   // Write position
    volatile uint16_t tail;   // Read position
    volatile uint16_t count;  // Number of items in buffer
} ring_buffer_t;

/**
 * @brief Initialize a ring buffer
 * @param rb Pointer to ring buffer structure
 * @param buffer Memory to use for the buffer
 * @param size Size of the buffer
 * @return error_code_t Error code
 */
error_code_t ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t size);

/**
 * @brief Push a byte to the ring buffer
 * @param rb Pointer to ring buffer structure
 * @param data Byte to push
 * @return error_code_t Error code
 */
error_code_t ring_buffer_push(ring_buffer_t *rb, uint8_t data);

/**
 * @brief Pop a byte from the ring buffer
 * @param rb Pointer to ring buffer structure
 * @param data Pointer to store popped byte
 * @return error_code_t Error code
 */
error_code_t ring_buffer_pop(ring_buffer_t *rb, uint8_t *data);

/**
 * @brief Check if ring buffer is empty
 * @param rb Pointer to ring buffer structure
 * @return true if empty, false otherwise
 */
bool ring_buffer_is_empty(const ring_buffer_t *rb);

/**
 * @brief Check if ring buffer is full
 * @param rb Pointer to ring buffer structure
 * @return true if full, false otherwise
 */
bool ring_buffer_is_full(const ring_buffer_t *rb);

/**
 * @brief Get number of items in ring buffer
 * @param rb Pointer to ring buffer structure
 * @return Number of items
 */
uint16_t ring_buffer_count(const ring_buffer_t *rb);

/**
 * @brief Clear the ring buffer
 * @param rb Pointer to ring buffer structure
 */
void ring_buffer_clear(ring_buffer_t *rb);

#endif // RING_BUFFER_H
