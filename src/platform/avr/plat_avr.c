#include "plat_avr.h"
#include "../../utils/common/error_codes.h"
#include <avr/eeprom.h>
#include <avr/wdt.h>

error_code_t plat_avr_init(void)
{
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
  wdt_enable(WDTO_2S); // 2 second timeout

  // Platform is initialized but interrupts remain disabled
  // The application layer should enable interrupts when ready
  return ERR_SUCCESS;
}

error_code_t plat_avr_deinit(void)
{
  // Disable interrupts
  cli();

  // Enable watchdog with minimum timeout
  wdt_enable(WDTO_15MS);

  // Wait for reset
  while (1)
    ;

  return ERR_SUCCESS;
}

void plat_avr_enable_interrupts(void) { sei(); }

void plat_avr_disable_interrupts(void) { cli(); }

void plat_avr_enter_sleep_mode(uint8_t mode)
{
  set_sleep_mode(mode);
  sleep_enable();
  sleep_mode();
  sleep_disable();
}

void plat_avr_exit_sleep_mode(void) { sleep_disable(); }

error_code_t plat_avr_set_clock_prescaler(uint8_t prescaler)
{
  if (prescaler > 8) {
    return ERR_CLOCK_INVALID_PRESCALER;
  }
  clock_prescale_set(prescaler);
  return ERR_SUCCESS;
}

uint32_t plat_avr_get_cpu_frequency(void) { return F_CPU; }

void plat_avr_watchdog_reset(void) { wdt_reset(); }

void plat_avr_watchdog_disable(void)
{
  cli(); // Disable interrupts
  wdt_reset();
  MCUSR &= ~(1 << WDRF);              // Clear watchdog reset flag
  WDTCSR |= (1 << WDCE) | (1 << WDE); // Start timed sequence
  WDTCSR = 0x00;                      // Disable watchdog
  sei();                              // Re-enable interrupts
}

error_code_t plat_avr_watchdog_enable(uint8_t timeout)
{
  uint8_t sreg = SREG; // Save interrupt state
  cli();               // Disable interrupts

  error_code_t status = ERR_SUCCESS;

  // Validate timeout value
  if (timeout > WDTO_8S) {
    status = ERR_INVALID_PARAM;
    goto exit;
  }

  wdt_reset();
  WDTCSR |= (1 << WDCE) | (1 << WDE); // Start timed sequence
  WDTCSR = timeout;                   // Set new timeout

exit:
  SREG = sreg; // Restore original interrupt state
  return status;
}

uint16_t plat_avr_get_stack_pointer(void) { return SP; }

uint16_t plat_avr_get_free_ram(void)
{
  extern uint16_t __heap_start, *__brkval;
  uint16_t v;
  return (uint16_t)&v - (__brkval == 0 ? (uint16_t)&__heap_start : (uint16_t)__brkval);
}

void plat_avr_soft_reset(void)
{
  wdt_enable(WDTO_15MS);
  while (1)
    ;
}

uint8_t plat_avr_get_reset_cause(void) { return MCUSR; }

error_code_t plat_avr_eeprom_write(uint16_t addr, const void *data, size_t size)
{
  if (!data || size == 0) {
    return ERR_INVALID_PARAM;
  }

  // Check if address + size exceeds EEPROM size
  if (addr + size > E2END + 1) {
    return ERR_OUT_OF_RANGE;
  }

  uint8_t sreg = SREG; // Save interrupt state
  cli();               // Disable interrupts during EEPROM write

  eeprom_write_block(data, (void *)(addr), size);

  SREG = sreg; // Restore interrupt state
  return ERR_SUCCESS;
}

error_code_t plat_avr_eeprom_read(uint16_t addr, void *data, size_t size)
{
  if (!data || size == 0) {
    return ERR_INVALID_PARAM;
  }

  // Check if address + size exceeds EEPROM size
  if (addr + size > E2END + 1) {
    return ERR_OUT_OF_RANGE;
  }

  uint8_t sreg = SREG; // Save interrupt state
  cli();               // Disable interrupts during EEPROM read

  eeprom_read_block(data, (const void *)(addr), size);

  SREG = sreg; // Restore interrupt state
  return ERR_SUCCESS;
}

error_code_t plat_avr_eeprom_write_byte(uint16_t addr, uint8_t data)
{
  if (addr > E2END) {
    return ERR_OUT_OF_RANGE;
  }

  uint8_t sreg = SREG; // Save interrupt state
  cli();               // Disable interrupts during EEPROM write

  eeprom_write_byte((const void *)(addr), data);

  SREG = sreg; // Restore interrupt state
  return ERR_SUCCESS;
}

error_code_t plat_avr_eeprom_read_byte(uint16_t addr, uint8_t *data)
{
  if (!data || addr > E2END) {
    return ERR_OUT_OF_RANGE;
  }

  uint8_t sreg = SREG; // Save interrupt state
  cli();               // Disable interrupts during EEPROM read

  *data = eeprom_read_byte((const void *)(addr));

  SREG = sreg; // Restore interrupt state
  return ERR_SUCCESS;
}
