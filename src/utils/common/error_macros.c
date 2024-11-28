#include "error_macros.h"
#include <avr/io.h>
#include <util/atomic.h>

// Initialize thread-local error context
error_context_t* current_error_context = NULL;

// Error context stack for nested try/catch
static error_context_t* error_context_stack[8];
static uint8_t error_context_stack_idx = 0;

void error_push_context(error_context_t* ctx) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (error_context_stack_idx < sizeof(error_context_stack)/sizeof(error_context_stack[0])) {
            error_context_stack[error_context_stack_idx++] = ctx;
        }
    }
}

error_context_t* error_pop_context(void) {
    error_context_t* ctx = NULL;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (error_context_stack_idx > 0) {
            ctx = error_context_stack[--error_context_stack_idx];
        }
    }
    return ctx;
}

error_context_t* error_get_current_context(void) {
    error_context_t* ctx = NULL;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (error_context_stack_idx > 0) {
            ctx = error_context_stack[error_context_stack_idx - 1];
        }
    }
    return ctx;
}
