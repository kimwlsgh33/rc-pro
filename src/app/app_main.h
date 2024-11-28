#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "../config/cfg_system.h"
#include "../config/cfg_debug.h"
#include "../config/cfg_hardware.h"
#include "../drivers/mcu/uart/drv_uart1.h"
#include "../drivers/mcu/uart/drv_uart2.h"
#include "../drivers/mcu/uart/drv_uart3.h"
#include "../utils/common/error_codes.h"

// Application States
typedef enum {
    RMC_ST_UNKNOWN = 'u',
    RMC_ST_INIT = 'i',
    RMC_ST_READY = 'R',
    RMC_ST_RUNNING = 'r',
    RMC_ST_ERROR = 'E'
} rmc_state_t;

// Application Status
typedef struct {
    rmc_state_t state;      // Current state
    uint8_t rstate;         // Running state
    char door;              // Door state
    uint16_t monotorPosition;
    uint16_t plasticCups;
    uint16_t paperCups;
    uint8_t purifiedTankLevel;
    uint8_t wasteTankLevel;
    uint8_t errorCode;
    uint8_t errorRoboSeq;
} RmcState;

// Operation Event
typedef struct {
    uint8_t cupMaterial;
} RcOpcEvent;

// Function declarations
error_code_t rm_init(void);
void rm_process(void);
bool rm_is_sensor_updated(void);
uint16_t rm_get_sensorValue(void);
RmcState rm_get_state(void);
bool rc_is_opc_event_updated(void);
RcOpcEvent rc_get_opcEvent(void);
error_code_t rm_mon_move_to(uint16_t position);
error_code_t rm_start(uint8_t mode);
error_code_t rm_stop(void);
error_code_t rm_reset_rc(void);
void rm_gd_open(void);
void rm_gd_open2(void);
void rm_gd_close(void);
void rm_gd_stop(void);
void rm_print_state(void);
void rm_init_position(void);
error_code_t us_process(void);

#endif // APP_MAIN_H
