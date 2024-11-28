#ifndef PLAT_AVR_H
#define PLAT_AVR_H

#include "../../config/cfg_system.h"
#include "../../utils/common/error_codes.h"

// AVR-specific includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>

// Platform initialization
error_code_t plat_avr_init(void);
error_code_t plat_avr_deinit(void);

// Interrupt management
void plat_avr_enable_interrupts(void);
void plat_avr_disable_interrupts(void);

// Power management
void plat_avr_enter_sleep_mode(uint8_t mode);
void plat_avr_exit_sleep_mode(void);

// Clock management
error_code_t plat_avr_set_clock_prescaler(uint8_t prescaler);
uint32_t plat_avr_get_cpu_frequency(void);

// Watchdog management
error_code_t plat_avr_watchdog_enable(uint8_t timeout);
void plat_avr_watchdog_disable(void);
void plat_avr_watchdog_reset(void);

// Memory management
uint16_t plat_avr_get_stack_pointer(void);
uint16_t plat_avr_get_free_ram(void);

// EEPROM management
/**
 * @brief Platform EEPROM functions
 */
error_code_t plat_avr_eeprom_write(uint16_t addr, const void *data, size_t size);
error_code_t plat_avr_eeprom_read(uint16_t addr, void *data, size_t size);
error_code_t plat_avr_eeprom_write_byte(uint16_t addr, uint8_t data);
error_code_t plat_avr_eeprom_read_byte(uint16_t addr, uint8_t *data);

// System control
void plat_avr_soft_reset(void);
uint8_t plat_avr_get_reset_cause(void);

#endif // PLAT_AVR_H
