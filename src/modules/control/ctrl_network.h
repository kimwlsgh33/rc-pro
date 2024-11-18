#ifndef __NETWORK_COMM_H__
#define __NETWORK_COMM_H__

#include "types.h"
#include <stdint.h>

// Network states
#define NET_STATE_DISCONNECTED  0x00
#define NET_STATE_CONNECTING    0x01
#define NET_STATE_CONNECTED     0x02
#define NET_STATE_ERROR         0x03

// Message types
#define NET_MSG_STATUS          0x10  // System status update
#define NET_MSG_ERROR           0x11  // Error notification
#define NET_MSG_CONFIG          0x12  // Configuration update
#define NET_MSG_COMMAND         0x13  // Remote command
#define NET_MSG_RESPONSE        0x14  // Command response

// Network configuration
typedef struct {
    uint8_t auto_connect;       // Auto-connect on startup
    uint8_t report_interval;    // Status report interval (seconds)
    uint8_t retry_count;        // Connection retry count
    uint16_t retry_interval;    // Retry interval (ms)
} NetworkConfig;

// Status packet structure
typedef struct {
    uint8_t msg_type;          // Message type
    uint8_t system_state;      // Current system state
    uint8_t error_code;        // Current error code
    uint8_t power_state;       // Power management state
    uint8_t component_health;  // Overall component health
    uint16_t plastic_cups;     // Number of plastic cups
    uint16_t paper_cups;       // Number of paper cups
    uint8_t tank_levels[2];    // Purified and waste tank levels
} StatusPacket;

// Function declarations
void net_init(void);
void net_set_config(NetworkConfig* config);
void net_get_config(NetworkConfig* config);
uint8_t net_get_state(void);
void net_connect(void);
void net_disconnect(void);
void net_send_status(void);
void net_process_command(uint8_t* data, uint16_t len);
void net_process_cycle(void);

#endif // __NETWORK_COMM_H__
