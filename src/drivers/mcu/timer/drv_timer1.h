#ifndef DRV_TIMER1_H
#define DRV_TIMER1_H

#include "drv_timer_common.h"
#include <stdint.h>

// Timer1 specific configuration structure
typedef struct {
    timer_base_config_t base_config;  // Add base configuration
    uint8_t external_clock_filter;  // External clock input filter for Timer1
    uint8_t input_capture_noise_canceler; // Timer1-specific noise canceler
    bool input_capture_edge_select;       // Timer1-specific edge selection
} timer1_specific_config_t;

// Timer1 interface functions
const timer_ops_t* timer1_get_ops(void);
const timer_hal_t* timer1_get_hal(void);

#endif // DRV_TIMER1_H
