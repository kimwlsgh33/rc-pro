#ifndef ERROR_MACROS_H
#define ERROR_MACROS_H

#include <setjmp.h>
#include "error_codes.h"

// Error handling context
typedef struct {
    jmp_buf jmp_buf;
    error_code_t error_code;
} error_context_t;

// Thread-local error context
extern error_context_t* current_error_context;

// Error handling macros
#define __try \
    do { \
        error_context_t error_ctx = {0}; \
        error_context_t* prev_ctx = current_error_context; \
        current_error_context = &error_ctx; \
        if (setjmp(error_ctx.jmp_buf) == 0) \

#define __catch \
        else { \
            error_code_t __error = error_ctx.error_code; \
            current_error_context = prev_ctx; \

#define __throw(error_code) \
    do { \
        if (current_error_context) { \
            current_error_context->error_code = (error_code); \
            longjmp(current_error_context->jmp_buf, 1); \
        } \
    } while (0)

#define __end_try \
        } \
        current_error_context = prev_ctx; \
    } while (0)

#endif // ERROR_MACROS_H
