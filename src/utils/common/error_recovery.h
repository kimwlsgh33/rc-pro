#ifndef ERROR_RECOVERY_H
#define ERROR_RECOVERY_H

#include "error_codes.h"
#include <stdbool.h>

// Recovery Statistics Structure
typedef struct {
    uint32_t attempts;
    uint32_t successes;
    uint32_t failures;
    uint32_t last_error;
    uint32_t last_attempt_time;
} recovery_stats_t;

// Recovery Handler Function Type
typedef error_code_t (*error_recovery_handler_t)(error_code_t error);

// Initialize the error recovery system
void error_recovery_init(void);

// Register an error recovery handler
error_code_t error_recovery_register_handler(error_code_t error_code,
                                           error_recovery_handler_t handler,
                                           uint8_t priority);

// Attempt to recover from an error
error_code_t error_recovery_attempt(error_code_t error);

// Get recovery statistics
const recovery_stats_t* error_recovery_get_stats(void);

// Check if recovery is currently in progress
bool error_recovery_is_active(void);

// Reset recovery statistics
void error_recovery_reset_stats(void);

// Register default recovery handlers
void error_recovery_register_defaults(void);

// Additional error recovery registration
void error_recovery_register_extended(void);

// ADC Error Recovery
#define ADC_ERROR_TIMEOUT      0x01
#define ADC_ERROR_CONVERSION   0x02
#define ADC_ERROR_REFERENCE    0x04
#define ADC_ERROR_CHANNEL      0x08

// UART Error Recovery
#define UART_ERROR_OVERFLOW    0x01
#define UART_ERROR_FRAMING     0x02
#define UART_ERROR_PARITY      0x04
#define UART_ERROR_BUFFER      0x08
#define UART_ERROR_DMA         0x10

// PWM Error Recovery
#define PWM_ERROR_CONFIG       0x01
#define PWM_ERROR_DUTY_CYCLE   0x02
#define PWM_ERROR_FREQUENCY    0x04
#define PWM_ERROR_CHANNEL      0x08

// Sensor Error Recovery
#define SENSOR_ERROR_TIMEOUT   0x01
#define SENSOR_ERROR_READING   0x02
#define SENSOR_ERROR_CALIBRATION 0x04
#define SENSOR_ERROR_RANGE     0x08

// DMA Error Recovery
#define DMA_ERROR_TRANSFER     0x01
#define DMA_ERROR_CONFIG       0x02
#define DMA_ERROR_BUFFER       0x04
#define DMA_ERROR_ALIGNMENT    0x08

// Stepper Motor Error Recovery
#define STEPPER_ERROR_STALL       0x01
#define STEPPER_ERROR_POSITION    0x02
#define STEPPER_ERROR_SPEED       0x04
#define STEPPER_ERROR_LIMIT       0x08
#define STEPPER_ERROR_DRIVER      0x10
#define STEPPER_ERROR_CURRENT     0x20

// Position Tracking Error Recovery
#define POSITION_ERROR_SYNC       0x01
#define POSITION_ERROR_OVERFLOW   0x02
#define POSITION_ERROR_LIMIT      0x04
#define POSITION_ERROR_SENSOR     0x08
#define POSITION_ERROR_CALIBRATION 0x10

// Additional recovery functions
error_code_t error_recovery_adc(uint8_t error_flags);
error_code_t error_recovery_uart(uint8_t error_flags);
error_code_t error_recovery_pwm(uint8_t error_flags);
error_code_t error_recovery_sensor(uint8_t error_flags);
error_code_t error_recovery_dma(uint8_t error_flags);
error_code_t error_recovery_stepper(uint8_t error_flags);
error_code_t error_recovery_position(uint8_t error_flags);

#endif // ERROR_RECOVERY_H
