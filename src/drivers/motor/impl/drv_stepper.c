#include "drv_stepper.h"
#include "../hal/drv_stepper_pins.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

// Static instance of active stepper driver
static motor_driver_t *active_stepper = NULL;

// Forward declarations of interface functions
static error_code_t stepper_init_fn(motor_driver_t *driver);
static error_code_t stepper_deinit_fn(motor_driver_t *driver);
static error_code_t stepper_enable_fn(motor_driver_t *driver);
static error_code_t stepper_disable_fn(motor_driver_t *driver);
static error_code_t stepper_move_to_fn(motor_driver_t *driver, int32_t position);
static error_code_t stepper_set_speed_fn(motor_driver_t *driver, uint16_t speed);
static error_code_t stepper_stop_fn(motor_driver_t *driver);
static error_code_t stepper_emergency_stop_fn(motor_driver_t *driver);
static error_code_t stepper_reset_error_fn(motor_driver_t *driver);
static void stepper_process_fn(motor_driver_t *driver);

// Static interface vtable
static const motor_interface_t stepper_interface = {.init           = stepper_init_fn,
                                                    .deinit         = stepper_deinit_fn,
                                                    .enable         = stepper_enable_fn,
                                                    .disable        = stepper_disable_fn,
                                                    .move_to        = stepper_move_to_fn,
                                                    .set_speed      = stepper_set_speed_fn,
                                                    .stop           = stepper_stop_fn,
                                                    .emergency_stop = stepper_emergency_stop_fn,
                                                    .reset_error    = stepper_reset_error_fn,
                                                    .process        = stepper_process_fn};

// Helper functions
static inline stepper_specific_t *get_specific(motor_driver_t *driver)
{
  return (stepper_specific_t *)driver->specific_data;
}

static inline const stepper_specific_t *get_specific_const(const motor_driver_t *driver)
{
  return (const stepper_specific_t *)driver->specific_data;
}

// Driver creation/destruction
error_code_t stepper_create(uint8_t id, const stepper_config_t *config, motor_driver_t **driver)
{
  if (!config || !driver) {
    return ERR_INVALID_PARAM;
  }

  // Allocate driver structure
  motor_driver_t *new_driver = (motor_driver_t *)malloc(sizeof(motor_driver_t));
  if (!new_driver) {
    return ERR_OUT_OF_MEMORY;
  }

  // Allocate specific data
  stepper_specific_t *specific = (stepper_specific_t *)malloc(sizeof(stepper_specific_t));
  if (!specific) {
    free(new_driver);
    return ERR_OUT_OF_MEMORY;
  }

  // Initialize driver structure
  new_driver->id                     = id;
  new_driver->config                 = config->base;
  new_driver->status.state           = MOTOR_STATE_UNKNOWN;
  new_driver->status.position        = 0;
  new_driver->status.target_position = 0;
  new_driver->status.current_speed   = 0;
  new_driver->status.target_speed    = 0;
  new_driver->status.direction       = MOTOR_DIR_CW;
  new_driver->status.error           = MOTOR_ERROR_NONE;
  new_driver->status.is_initialized  = false;
  new_driver->status.is_enabled      = false;
  new_driver->vtable                 = &stepper_interface;
  new_driver->specific_data          = specific;

  // Initialize specific data
  specific->step_count        = 0;
  specific->is_homed          = false;
  specific->limit_switch_hit  = false;
  specific->current_mode      = config->base.micro_stepping;
  specific->missed_step_count = 0;
  specific->last_step_time    = 0;
  specific->stall_detected    = false;

  // Initialize status
  new_driver->status.state = MOTOR_STATE_READY;
  new_driver->status.door_state = DOOR_STATE_UNKNOWN;

  // Set output parameter
  *driver = new_driver;

  // Set as active stepper if none is set yet
  if (!active_stepper) {
    active_stepper = new_driver;
  }

  return ERR_SUCCESS;
}

error_code_t stepper_destroy(motor_driver_t *driver)
{
  if (!driver) {
    return ERR_INVALID_PARAM;
  }

  // Clear active stepper if this is it
  if (active_stepper == driver) {
    active_stepper = NULL;
  }

  // Free specific data
  if (driver->specific_data) {
    free(driver->specific_data);
  }

  // Free driver structure
  free(driver);
  return ERR_SUCCESS;
}

// Interface implementations
error_code_t stepper_init_fn(motor_driver_t *driver)
{
  if (!driver) {
    return ERR_INVALID_PARAM;
  }

  if (driver->status.is_initialized) {
    return ERR_ALREADY_INIT;
  }

  // Initialize hardware pins
  // Set step and direction pins as outputs
  DDRB |= (1 << STEP_PIN) | (1 << DIR_PIN);
  // Set enable and MS pins as outputs
  DDRD |= (1 << EN_PIN) | (1 << MS1_PIN) | (1 << MS2_PIN) | (1 << MS3_PIN);
  // Set limit switch pin as input with pull-up
  DDRB &= ~(1 << LIMIT_PIN);
  PORTB |= (1 << LIMIT_PIN);

  // Initialize timer for step generation
  // Using Timer1 in CTC mode for precise step timing
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);  // CTC mode
  TIMSK1 = (1 << OCIE1A); // Enable compare match interrupt

  // Default to disabled state
  PORTD |= (1 << EN_PIN); // Active low enable

  driver->status.is_initialized = true;
  driver->status.state          = MOTOR_STATE_STOPPED;
  return ERR_SUCCESS;
}

error_code_t stepper_deinit_fn(motor_driver_t *driver)
{
  if (!driver) {
    return ERR_INVALID_PARAM;
  }

  // TODO: Deinitialize hardware

  driver->status.state          = MOTOR_STATE_UNKNOWN;
  driver->status.is_initialized = false;
  return ERR_SUCCESS;
}

error_code_t stepper_enable_fn(motor_driver_t *driver)
{
  if (!driver) {
    return ERR_INVALID_PARAM;
  }

  if (!driver->status.is_initialized) {
    return ERR_MOTOR_NOT_INIT;
  }

  if (driver->status.is_enabled) {
    return ERR_ALREADY_INIT;
  }

  // TODO: Enable stepper driver hardware

  driver->status.is_enabled = true;
  return ERR_SUCCESS;
}

error_code_t stepper_disable_fn(motor_driver_t *driver)
{
  if (!driver) {
    return ERR_INVALID_PARAM;
  }

  if (!driver->status.is_initialized) {
    return ERR_MOTOR_NOT_INIT;
  }

  if (!driver->status.is_enabled) {
    return ERR_MOTOR_NOT_ENABLED;
  }

  // TODO: Disable stepper driver hardware

  driver->status.is_enabled = false;
  return ERR_SUCCESS;
}

error_code_t stepper_move_to_fn(motor_driver_t *driver, int32_t position)
{
  if (!driver) {
    return ERR_INVALID_PARAM;
  }

  if (!driver->status.is_initialized) {
    return ERR_MOTOR_NOT_INIT;
  }

  if (!driver->status.is_enabled) {
    return ERR_MOTOR_NOT_ENABLED;
  }

  stepper_specific_t *specific = get_specific(driver);

  // Check if we're already at the target position
  if (driver->status.position == position) {
    return ERR_SUCCESS;
  }

  // Update direction based on target position
  if (position > driver->status.position) {
    driver->status.direction = MOTOR_DIR_CW;
    driver->status.door_state = DOOR_STATE_OPENING;
  } else {
    driver->status.direction = MOTOR_DIR_CCW;
    driver->status.door_state = DOOR_STATE_CLOSING;
  }

  // Update target position and state
  driver->status.target_position = position;
  driver->status.state = MOTOR_STATE_MOVING;

  // Start timer with initial speed
  if (driver->status.current_speed == 0) {
    driver->status.current_speed = driver->config.min_speed;
  }
  uint16_t timer_count = F_CPU / (2 * driver->status.current_speed) - 1;
  OCR1A = timer_count;
  TCCR1B |= (1 << CS11); // Start timer with prescaler 8

  return ERR_SUCCESS;
}

error_code_t stepper_set_speed_fn(motor_driver_t *driver, uint16_t speed)
{
  if (!driver) {
    return ERR_INVALID_PARAM;
  }

  if (!driver->status.is_initialized) {
    return ERR_MOTOR_NOT_INIT;
  }

  if (speed > driver->config.max_speed) {
    speed = driver->config.max_speed;
  }

  driver->status.target_speed = speed;

  // TODO: Update hardware timer/PWM for new speed

  return ERR_SUCCESS;
}

error_code_t stepper_stop_fn(motor_driver_t *driver)
{
  if (!driver) {
    return ERR_INVALID_PARAM;
  }

  if (!driver->status.is_initialized) {
    return ERR_MOTOR_NOT_INIT;
  }

  if (!driver->status.is_enabled) {
    return ERR_MOTOR_NOT_ENABLED;
  }

  // Stop timer
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
  PORTD |= (1 << EN_PIN);

  // Update state
  driver->status.state           = MOTOR_STATE_STOPPED;
  driver->status.target_position = driver->status.position; // We've reached target

  // Update door state based on position
  if (driver->status.position == 0) {
    driver->status.door_state = DOOR_STATE_CLOSED;
  } else if (driver->status.position >= driver->config.max_speed) { // Using max_speed as proxy for fully open
    driver->status.door_state = DOOR_STATE_OPEN;
  } else {
    driver->status.door_state = DOOR_STATE_PARTIALLY_OPEN;
  }

  return ERR_SUCCESS;
}

error_code_t stepper_emergency_stop_fn(motor_driver_t *driver)
{
  if (!driver) {
    return ERR_INVALID_PARAM;
  }

  // Immediately stop timer
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));

  // Disable driver
  PORTD |= (1 << EN_PIN);

  // Update state
  driver->status.state           = MOTOR_STATE_ERROR;
  driver->status.target_position = driver->status.position; // Stop where we are
  driver->status.error           = MOTOR_ERROR_SYSTEM; // Emergency stop is a system-level error

  stepper_specific_t *specific = get_specific(driver);

  return ERR_SUCCESS;
}

error_code_t stepper_reset_error_fn(motor_driver_t *driver)
{
  if (!driver) {
    return ERR_INVALID_PARAM;
  }

  stepper_specific_t *specific = get_specific(driver);
  if (driver->status.state == MOTOR_STATE_ERROR) {
    driver->status.error        = MOTOR_ERROR_NONE;
    driver->status.state        = MOTOR_STATE_READY;
    specific->stall_detected    = false;
    specific->missed_step_count = 0;
    specific->last_step_time    = 0;
  }

  return ERR_SUCCESS;
}

static void stepper_process_fn(motor_driver_t *driver)
{
  if (!driver || !driver->status.is_enabled) {
    return;
  }

  stepper_specific_t *specific = get_specific(driver);

  // Process movement
  if (driver->status.state == MOTOR_STATE_MOVING) {
    int32_t remaining_steps = abs(driver->status.target_position - driver->status.position);

    // Calculate deceleration point
    // v² = u² + 2as (where v=0 at end)
    // Therefore s = u²/2a where s is stopping distance
    uint32_t stopping_distance = (driver->status.current_speed * driver->status.current_speed) /
                                 (2 * driver->config.deceleration);

    if (remaining_steps > stopping_distance) {
      // Accelerate or maintain speed
      if (driver->status.current_speed < driver->status.target_speed) {
        // v = u + at
        driver->status.current_speed += driver->config.acceleration;
        if (driver->status.current_speed > driver->status.target_speed) {
          driver->status.current_speed = driver->status.target_speed;
        }
      }
    } else {
      // Decelerate
      if (driver->status.current_speed > 100) { // Minimum speed threshold
        driver->status.current_speed -= driver->config.deceleration;
      }
    }

    // Update timer frequency for new speed
    if (driver->status.current_speed > 0) {
      uint16_t timer_count = F_CPU / (2 * driver->status.current_speed) - 1;
      OCR1A                = timer_count;
    }
  }
}

// Stepper-specific operations
error_code_t stepper_home(motor_driver_t *driver)
{
  if (!driver || !driver->status.is_enabled) {
    return ERR_INVALID_STATE;
  }

  stepper_specific_t *specific = get_specific(driver);

  // TODO: Implement homing sequence
  // - Move towards home switch
  // - Detect home switch
  // - Set position to 0
  // - Update is_homed flag

  return ERR_SUCCESS;
}

error_code_t stepper_set_micro_stepping(motor_driver_t *driver, uint8_t micro_stepping)
{
  if (!driver || !driver->status.is_enabled) {
    return ERR_INVALID_STATE;
  }

  stepper_specific_t *specific = get_specific(driver);

  // Validate micro_stepping value (must be power of 2)
  if ((micro_stepping & (micro_stepping - 1)) != 0) {
    return ERR_INVALID_PARAM;
  }

  specific->current_mode = micro_stepping;

  // TODO: Configure hardware for new micro-stepping mode

  return ERR_SUCCESS;
}

error_code_t stepper_set_current(motor_driver_t *driver, uint8_t current)
{
  if (!driver || !driver->status.is_enabled) {
    return ERR_INVALID_STATE;
  }

  if (current > driver->config.current_limit) {
    return ERR_INVALID_PARAM; // Current exceeds configured limit
  }

  // TODO: Configure hardware current limit

  return ERR_SUCCESS;
}

// Stepper-specific getters
bool stepper_is_homed(const motor_driver_t *driver)
{
  const stepper_specific_t *specific = get_specific_const(driver);
  return specific ? specific->is_homed : false;
}

bool stepper_limit_switch_hit(const motor_driver_t *driver)
{
  const stepper_specific_t *specific = get_specific_const(driver);
  return specific ? specific->limit_switch_hit : false;
}

uint32_t stepper_get_step_count(const motor_driver_t *driver)
{
  const stepper_specific_t *specific = get_specific_const(driver);
  return specific ? specific->step_count : 0;
}

bool stepper_is_stalled(const motor_driver_t *driver)
{
  const stepper_specific_t *specific = get_specific_const(driver);
  return specific ? specific->stall_detected : false;
}

// Get the currently active stepper instance
static motor_driver_t *get_active_stepper(void) { return active_stepper; }

// Timer1 Compare Match A ISR for step generation
ISR(TIMER1_COMPA_vect)
{
  // Toggle step pin
  PORTB ^= (1 << STEP_PIN);

  // Only process on rising edge
  if (PORTB & (1 << STEP_PIN)) {
    motor_driver_t *driver = get_active_stepper(); // Implement this to get current stepper instance
    if (driver) {
      stepper_specific_t *specific = get_specific(driver);
      uint16_t current_time        = TCNT1;

      // Check for stall condition
      if (specific->last_step_time > 0) {
        uint16_t step_duration = current_time - specific->last_step_time;
        if (step_duration > STEP_TIMING_WINDOW) {
          specific->missed_step_count++;
          if (specific->missed_step_count >= STALL_DETECTION_THRESHOLD) {
            specific->stall_detected = true;
            // Emergency stop on stall
            stepper_emergency_stop_fn(driver);
            driver->status.error = MOTOR_ERROR_STALL;
            return;
          }
        } else {
          // Reset counter if step timing is good
          specific->missed_step_count = 0;
        }
      }

      specific->last_step_time = current_time;

      // Update position based on direction
      if (driver->status.direction == MOTOR_DIR_CW) {
        driver->status.position++;
      } else {
        driver->status.position--;
      }

      specific->step_count++;

      // Check if we've reached target position
      if (driver->status.position == driver->status.target_position) {
        // Stop the motor
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
        // Disable driver
        PORTD |= (1 << EN_PIN);
        driver->status.state = MOTOR_STATE_STOPPED;
      }

      // Check limit switch
      if (PINB & (1 << LIMIT_PIN)) {
        specific->limit_switch_hit = true;
        // Emergency stop if limit switch hit
        stepper_emergency_stop_fn(driver);
      }
    }
  }
}
