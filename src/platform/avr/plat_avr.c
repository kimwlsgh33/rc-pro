#include <avr/wdt.h>
#include "plat_avr.h"
#include "error_codes.h"

error_code_t plat_avr_init(void) {
    // Disable interrupts during initialization
    cli();
    
    // Disable watchdog if enabled by bootloader
    wdt_disable();
    
    // Configure system clock
    #if F_CPU == 16000000UL
        clock_prescale_set(clock_div_1);
    #endif
    
    // Configure watchdog
    wdt_reset();
    wdt_enable(WDTO_2S);  // 2 second timeout
    
    // Platform is initialized but interrupts remain disabled
    // The application layer should enable interrupts when ready
    return ERROR_CODE_SUCCESS;
}

error_code_t plat_avr_deinit(void) {
    // Disable interrupts
    cli();
    
    // Enable watchdog with minimum timeout
    wdt_enable(WDTO_15MS);
    
    // Wait for reset
    while(1);
    
    return ERROR_CODE_SUCCESS;
}

void plat_avr_enable_interrupts(void) {
    sei();
}

void plat_avr_disable_interrupts(void) {
    cli();
}

void plat_avr_enter_sleep_mode(uint8_t mode) {
    set_sleep_mode(mode);
    sleep_enable();
    sleep_mode();
    sleep_disable();
}

void plat_avr_exit_sleep_mode(void) {
    sleep_disable();
}

error_code_t plat_avr_set_clock_prescaler(uint8_t prescaler) {
    if (prescaler > 8) {
        return ERROR_CODE_INVALID_PARAMETER;
    }
    clock_prescale_set(prescaler);
    return ERROR_CODE_SUCCESS;
}

uint32_t plat_avr_get_cpu_frequency(void) {
    return F_CPU;
}

void plat_avr_watchdog_reset(void) {
    wdt_reset();
}

void plat_avr_watchdog_disable(void) {
    cli();  // Disable interrupts
    wdt_reset();
    MCUSR &= ~(1<<WDRF);  // Clear watchdog reset flag
    WDTCSR |= (1<<WDCE) | (1<<WDE);  // Start timed sequence
    WDTCSR = 0x00;  // Disable watchdog
    sei();  // Re-enable interrupts
}

void plat_avr_watchdog_enable(uint8_t timeout) {
    cli();  // Disable interrupts
    wdt_reset();
    WDTCSR |= (1<<WDCE) | (1<<WDE);  // Start timed sequence
    WDTCSR = timeout;  // Set new timeout
    sei();  // Re-enable interrupts
}

uint16_t plat_avr_get_stack_pointer(void) {
    return SP;
}

uint16_t plat_avr_get_free_ram(void) {
    extern uint16_t __heap_start, *__brkval;
    uint16_t v;
    return (uint16_t) &v - (__brkval == 0 ? (uint16_t) &__heap_start : (uint16_t) __brkval);
}

void plat_avr_soft_reset(void) {
    wdt_enable(WDTO_15MS);
    while(1);
}

uint8_t plat_avr_get_reset_cause(void) {
    return MCUSR;
}
