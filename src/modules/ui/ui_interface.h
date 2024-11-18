#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "../../config/cfg_system.h"
#include "../../config/cfg_debug.h"
#include "../../platform/common/plat_types.h"
#include "../../utils/common/error_codes.h"
#include "../../utils/common/logging.h"
#include "../../core/module/mod_manager.h"
#include "../../core/event/evt_manager.h"

// LED Indicators
#define LED_POWER           0x01
#define LED_STATUS          0x02
#define LED_ERROR           0x03
#define LED_NETWORK         0x04
#define LED_TANK_LEVEL      0x05

// LED States
#define LED_OFF             0x00
#define LED_ON             0x01
#define LED_BLINK_SLOW     0x02  // 1Hz
#define LED_BLINK_FAST     0x03  // 2Hz
#define LED_PULSE          0x04  // Breathing effect

// Buzzer Patterns
#define BUZZ_NONE          0x00
#define BUZZ_SHORT         0x01  // 100ms beep
#define BUZZ_LONG          0x02  // 500ms beep
#define BUZZ_DOUBLE        0x03  // Two short beeps
#define BUZZ_ERROR         0x04  // Three short beeps
#define BUZZ_CRITICAL      0x05  // Continuous beep

// Display Modes
#define DISP_MODE_STATUS    0x00  // Normal status display
#define DISP_MODE_ERROR     0x01  // Error information
#define DISP_MODE_MENU      0x02  // Menu navigation
#define DISP_MODE_DEBUG     0x03  // Debug information

// Function declarations
void ui_init(void);
void ui_set_led(uint8_t led_id, uint8_t state);
void ui_set_buzzer(uint8_t pattern);
void ui_clear_buzzer(void);
void ui_update_display(uint8_t mode);
void ui_process_cycle(void);

// Status display functions
void ui_show_status(void);
void ui_show_error(uint8_t error_code);
void ui_show_tank_levels(void);
void ui_show_cup_counts(void);

// User feedback functions
void ui_indicate_operation_start(void);
void ui_indicate_operation_complete(void);
void ui_indicate_error(void);
void ui_indicate_warning(void);

#endif // UI_INTERFACE_H
