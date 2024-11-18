#ifndef DRV_TIMER1_H
#define DRV_TIMER1_H

#include "drv_timer.h"

/**
 * @brief Initialize Timer1 with configuration
 * 
 * @param config Timer configuration structure
 * @return error_code_t Error code
 */
error_code_t timer1_init(const timer_config_t* config);

/**
 * @brief Deinitialize Timer1
 * 
 * @return error_code_t Error code
 */
error_code_t timer1_deinit(void);

/**
 * @brief Get Timer1 driver interface
 * 
 * @return const timer_driver_t* Pointer to Timer1 driver interface
 */
const timer_driver_t* timer1_get_driver(void);

#endif // DRV_TIMER1_H
