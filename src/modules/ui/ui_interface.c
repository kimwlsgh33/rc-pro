#include "user_interface.h"
#include "putil.h"
#include "timer.h"
#include "error_handling.h"
#include <string.h>
#include <avr/io.h>

// Pin definitions for LEDs
#define LED_POWER_PIN       PORTB7  // On-board LED
#define LED_STATUS_PIN      PORTB6
#define LED_ERROR_PIN       PORTB5
#define LED_NETWORK_PIN     PORTB4
#define LED_TANK_PIN        PORTB3

// Pin definition for buzzer
#define BUZZER_PIN         PORTD7

// Timer IDs
static int led_timer_id = -1;
static int buzzer_timer_id = -1;
static int display_timer_id = -1;

// LED states
static uint8_t led_states[5] = {0};
static uint8_t led_blink_state = 0;

// Buzzer state
static uint8_t current_buzz_pattern = BUZZ_NONE;
static uint8_t buzz_repeat_count = 0;

// Display state
static uint8_t current_display_mode = DISP_MODE_STATUS;
static uint8_t display_update_needed = 1;

// Initialize user interface
void ui_init(void) {
    // Configure LED pins as outputs
    _SET_OUTPUT(DDRB, LED_POWER_PIN);
    _SET_OUTPUT(DDRB, LED_STATUS_PIN);
    _SET_OUTPUT(DDRB, LED_ERROR_PIN);
    _SET_OUTPUT(DDRB, LED_NETWORK_PIN);
    _SET_OUTPUT(DDRB, LED_TANK_PIN);
    
    // Configure buzzer pin as output
    _SET_OUTPUT(DDRD, BUZZER_PIN);
    
    // Initialize all LEDs to off
    _SET(PORTB, LED_POWER_PIN);
    _SET(PORTB, LED_STATUS_PIN);
    _SET(PORTB, LED_ERROR_PIN);
    _SET(PORTB, LED_NETWORK_PIN);
    _SET(PORTB, LED_TANK_PIN);
    
    // Initialize buzzer to off
    _CLR(PORTD, BUZZER_PIN);
    
    // Allocate timers
    led_timer_id = timer_alloc();
    buzzer_timer_id = timer_alloc();
    display_timer_id = timer_alloc();
    
    // Set initial timer values
    timer_set(led_timer_id, 500);      // 500ms for LED blinking
    timer_set(buzzer_timer_id, 100);   // 100ms for buzzer patterns
    timer_set(display_timer_id, 1000); // 1s for display updates
    
    // Set power LED on
    ui_set_led(LED_POWER, LED_ON);
}

// Set LED state
void ui_set_led(uint8_t led_id, uint8_t state) {
    if (led_id >= 1 && led_id <= 5) {
        led_states[led_id - 1] = state;
    }
}

// Update LED hardware state
static void update_led_hw(uint8_t led_id, uint8_t on) {
    switch (led_id) {
        case LED_POWER:
            if (on) _CLR(PORTB, LED_POWER_PIN);
            else _SET(PORTB, LED_POWER_PIN);
            break;
        case LED_STATUS:
            if (on) _CLR(PORTB, LED_STATUS_PIN);
            else _SET(PORTB, LED_STATUS_PIN);
            break;
        case LED_ERROR:
            if (on) _CLR(PORTB, LED_ERROR_PIN);
            else _SET(PORTB, LED_ERROR_PIN);
            break;
        case LED_NETWORK:
            if (on) _CLR(PORTB, LED_NETWORK_PIN);
            else _SET(PORTB, LED_NETWORK_PIN);
            break;
        case LED_TANK_LEVEL:
            if (on) _CLR(PORTB, LED_TANK_PIN);
            else _SET(PORTB, LED_TANK_PIN);
            break;
    }
}

// Set buzzer pattern
void ui_set_buzzer(uint8_t pattern) {
    current_buzz_pattern = pattern;
    buzz_repeat_count = 0;
    
    // Start the pattern immediately
    switch (pattern) {
        case BUZZ_SHORT:
            _SET(PORTD, BUZZER_PIN);
            timer_set(buzzer_timer_id, 100);
            break;
        case BUZZ_LONG:
            _SET(PORTD, BUZZER_PIN);
            timer_set(buzzer_timer_id, 500);
            break;
        case BUZZ_DOUBLE:
            _SET(PORTD, BUZZER_PIN);
            timer_set(buzzer_timer_id, 100);
            buzz_repeat_count = 1;
            break;
        case BUZZ_ERROR:
            _SET(PORTD, BUZZER_PIN);
            timer_set(buzzer_timer_id, 100);
            buzz_repeat_count = 2;
            break;
        case BUZZ_CRITICAL:
            _SET(PORTD, BUZZER_PIN);
            timer_set(buzzer_timer_id, 1000);
            buzz_repeat_count = 255; // Continuous
            break;
        default:
            _CLR(PORTD, BUZZER_PIN);
            break;
    }
}

// Clear buzzer
void ui_clear_buzzer(void) {
    current_buzz_pattern = BUZZ_NONE;
    buzz_repeat_count = 0;
    _CLR(PORTD, BUZZER_PIN);
}

// Update display mode
void ui_update_display(uint8_t mode) {
    if (mode <= DISP_MODE_DEBUG) {
        current_display_mode = mode;
        display_update_needed = 1;
    }
}

// Show current status
void ui_show_status(void) {
    // TODO: Implement actual display output
    // For now, just use UART for debug output
    printf("System Status:\n");
    printf("State: %c\n", rmcState.state);
    printf("Plastic Cups: %d\n", rmcState.plasticCups);
    printf("Paper Cups: %d\n", rmcState.paperCups);
}

// Show error information
void ui_show_error(uint8_t error_code) {
    // TODO: Implement actual display output
    printf("Error: 0x%02X\n", error_code);
    printf("Description: %s\n", error_get_description(error_code));
}

// Show tank levels
void ui_show_tank_levels(void) {
    // TODO: Implement actual display output
    printf("Tank Levels:\n");
    printf("Purified: %d\n", rmcState.purifiedTankLevel);
    printf("Waste: %d\n", rmcState.wasteTankLevel);
}

// Show cup counts
void ui_show_cup_counts(void) {
    // TODO: Implement actual display output
    printf("Cup Counts:\n");
    printf("Plastic: %d\n", rmcState.plasticCups);
    printf("Paper: %d\n", rmcState.paperCups);
}

// Operation feedback functions
void ui_indicate_operation_start(void) {
    ui_set_led(LED_STATUS, LED_BLINK_FAST);
    ui_set_buzzer(BUZZ_SHORT);
}

void ui_indicate_operation_complete(void) {
    ui_set_led(LED_STATUS, LED_ON);
    ui_set_buzzer(BUZZ_DOUBLE);
}

void ui_indicate_error(void) {
    ui_set_led(LED_ERROR, LED_ON);
    ui_set_buzzer(BUZZ_ERROR);
    ui_update_display(DISP_MODE_ERROR);
}

void ui_indicate_warning(void) {
    ui_set_led(LED_ERROR, LED_BLINK_SLOW);
    ui_set_buzzer(BUZZ_DOUBLE);
}

// Main UI processing cycle
void ui_process_cycle(void) {
    // Process LED blinking
    if (timer_isfired(led_timer_id)) {
        led_blink_state = !led_blink_state;
        timer_set(led_timer_id, 500);
        
        // Update all LEDs
        for (uint8_t i = 0; i < 5; i++) {
            uint8_t led_id = i + 1;
            switch (led_states[i]) {
                case LED_ON:
                    update_led_hw(led_id, 1);
                    break;
                case LED_OFF:
                    update_led_hw(led_id, 0);
                    break;
                case LED_BLINK_SLOW:
                    update_led_hw(led_id, led_blink_state);
                    break;
                case LED_BLINK_FAST:
                    update_led_hw(led_id, !led_blink_state);
                    break;
                case LED_PULSE:
                    // TODO: Implement PWM for breathing effect
                    update_led_hw(led_id, led_blink_state);
                    break;
            }
        }
    }
    
    // Process buzzer patterns
    if (current_buzz_pattern != BUZZ_NONE && timer_isfired(buzzer_timer_id)) {
        if (_GET(PORTD, BUZZER_PIN)) {
            _CLR(PORTD, BUZZER_PIN);
            if (buzz_repeat_count > 0) {
                buzz_repeat_count--;
                timer_set(buzzer_timer_id, 100);
            } else {
                current_buzz_pattern = BUZZ_NONE;
            }
        } else if (buzz_repeat_count > 0) {
            _SET(PORTD, BUZZER_PIN);
            timer_set(buzzer_timer_id, 100);
        }
    }
    
    // Process display updates
    if (timer_isfired(display_timer_id)) {
        timer_set(display_timer_id, 1000);
        
        if (display_update_needed) {
            switch (current_display_mode) {
                case DISP_MODE_STATUS:
                    ui_show_status();
                    break;
                case DISP_MODE_ERROR:
                    ui_show_error(error_get_last());
                    break;
                case DISP_MODE_MENU:
                    // TODO: Implement menu display
                    break;
                case DISP_MODE_DEBUG:
                    // TODO: Implement debug display
                    break;
            }
            display_update_needed = 0;
        }
    }
}
