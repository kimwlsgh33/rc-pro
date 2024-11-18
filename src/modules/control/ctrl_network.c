#include "network_comm.h"
#include "putil.h"
#include "timer.h"
#include "uart2.h"  // Using UART2 for network communication
#include "error_handling.h"
#include "power_management.h"
#include <string.h>

#define NET_PACKET_START    0xAA
#define NET_PACKET_END      0x55
#define NET_MAX_PACKET_SIZE 64
#define NET_MIN_PACKET_SIZE 5
#define NET_PACKET_OVERHEAD 5
#define NET_DEFAULT_TIMEOUT 1000  // 1 second

static NetworkConfig net_config;
static uint8_t net_state = NET_STATE_DISCONNECTED;
static int net_timer_id = -1;
static uint32_t last_status_time = 0;
static uint8_t tx_buffer[NET_MAX_PACKET_SIZE];
static uint8_t rx_buffer[NET_MAX_PACKET_SIZE];
static uint16_t rx_index = 0;
static uint8_t retry_count = 0;

// Initialize network communication
void net_init(void) {
    // Set default configuration
    net_config.auto_connect = 1;
    net_config.report_interval = 5;  // 5 seconds
    net_config.retry_count = 3;
    net_config.retry_interval = 5000;  // 5 seconds
    
    // Initialize UART for network communication
    uart2_init(115200);  // Higher baud rate for network comm
    
    // Allocate timer for network operations
    net_timer_id = timer_alloc();
    timer_set(net_timer_id, NET_DEFAULT_TIMEOUT);
    
    // Auto-connect if enabled
    if (net_config.auto_connect) {
        net_connect();
    }
}

void net_set_config(NetworkConfig* config) {
    memcpy(&net_config, config, sizeof(NetworkConfig));
}

void net_get_config(NetworkConfig* config) {
    memcpy(config, &net_config, sizeof(NetworkConfig));
}

uint8_t net_get_state(void) {
    return net_state;
}

// Calculate checksum for packet
static uint8_t calculate_checksum(uint8_t* data, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

// Send a network packet
static error_code_t send_packet(uint8_t* data, uint16_t len) {
    uint8_t checksum = calculate_checksum(data, len);
    
    uart2_putc(NET_PACKET_START);
    uart2_putc(len & 0xFF);
    uart2_putc((len >> 8) & 0xFF);
    
    for (uint16_t i = 0; i < len; i++) {
        uart2_putc(data[i]);
    }
    
    uart2_putc(checksum);
    uart2_putc(NET_PACKET_END);
    
    return ERROR_CODE_SUCCESS;
}

void net_connect(void) {
    if (net_state == NET_STATE_DISCONNECTED) {
        net_state = NET_STATE_CONNECTING;
        retry_count = 0;
        
        // Send connection request
        if (send_connect_packet() != ERROR_CODE_SUCCESS) {
            error_record(ERR_NETWORK_SEND);
        }
        
        timer_set(net_timer_id, net_config.retry_interval);
    }
}

void net_disconnect(void) {
    if (net_state == NET_STATE_CONNECTED) {
        // Send disconnect notification
        tx_buffer[0] = NET_MSG_COMMAND;
        tx_buffer[1] = 0x02;  // Disconnect command
        send_packet(tx_buffer, 2);
        
        net_state = NET_STATE_DISCONNECTED;
    }
}

void net_send_status(void) {
    if (net_state != NET_STATE_CONNECTED) {
        return;
    }
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        StatusPacket status;
        status.msg_type = NET_MSG_STATUS;
        status.system_state = rmcState.state;
        status.error_code = error_get_last();
        status.power_state = pwr_get_state();
        status.component_health = pm_get_health_status(0);  // Overall health
        status.plastic_cups = rmcState.plasticCups;
        status.paper_cups = rmcState.paperCups;
        status.tank_levels[0] = rmcState.purifiedTankLevel;
        status.tank_levels[1] = rmcState.wasteTankLevel;
        
        // Cache the packet locally before sending
        uint8_t packet_buffer[sizeof(StatusPacket)];
        memcpy(packet_buffer, &status, sizeof(StatusPacket));
        
        // Update timestamp before releasing lock
        last_status_time = timer_get(net_timer_id);
        
        // Send packet after releasing lock to prevent deadlock
        if (send_packet(packet_buffer, sizeof(StatusPacket)) != ERROR_CODE_SUCCESS) {
            error_record(ERR_NETWORK_STATUS);
        }
    }
}

void net_process_command(uint8_t* data, uint16_t len) {
    if (len < 2) return;
    
    switch (data[1]) {  // Command type
        case 0x01:  // Connect response
            if (data[2] == 0x00) {  // Success
                net_state = NET_STATE_CONNECTED;
                retry_count = 0;
            } else {
                if (++retry_count >= net_config.retry_count) {
                    net_state = NET_STATE_ERROR;
                } else {
                    timer_set(net_timer_id, net_config.retry_interval);
                }
            }
            break;
            
        case 0x03:  // Configuration update
            if (len >= sizeof(NetworkConfig) + 2) {
                NetworkConfig* new_config = (NetworkConfig*)(data + 2);
                net_set_config(new_config);
            }
            break;
            
        case 0x04:  // Remote command
            // Handle remote commands here
            break;
    }
}

void net_process_cycle(void) {
    uint32_t current_time = timer_get(net_timer_id);
    
    // Process received data
    while (uart2_available()) {
        uint8_t c = uart2_getc();
        
        // Validate packet start
        if (rx_index == 0 && c != NET_PACKET_START) {
            continue;  // Wait for packet start
        }
        
        // Check buffer space before adding byte
        if (rx_index >= NET_MAX_PACKET_SIZE) {
            error_record(ERR_BUFFER_OVERFLOW);
            rx_index = 0;  // Reset buffer
            continue;
        }
        
        rx_buffer[rx_index++] = c;
        
        // Process complete packet with proper validation
        if (c == NET_PACKET_END && rx_index >= NET_MIN_PACKET_SIZE) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                // Validate packet length
                uint16_t claimed_len = (rx_buffer[1] | (rx_buffer[2] << 8));
                if (claimed_len + NET_PACKET_OVERHEAD == rx_index) {
                    // Validate checksum
                    uint8_t checksum = rx_buffer[rx_index - 2];
                    uint8_t calc_checksum = calculate_checksum(rx_buffer + 3, claimed_len);
                    
                    if (checksum == calc_checksum) {
                        net_process_command(rx_buffer + 3, claimed_len);
                    } else {
                        error_record(ERR_PACKET_CHECKSUM);
                    }
                } else {
                    error_record(ERR_PACKET_LENGTH);
                }
            }
            rx_index = 0;  // Reset for next packet
        }
    }
    
    // Connection retry logic with proper error handling
    if (net_state == NET_STATE_CONNECTING && timer_isfired(net_timer_id)) {
        if (retry_count < net_config.retry_count) {
            if (send_connect_packet() != ERROR_CODE_SUCCESS) {
                error_record(ERR_NETWORK_SEND);
            }
            retry_count++;
            timer_set(net_timer_id, net_config.retry_interval);
        } else {
            net_state = NET_STATE_ERROR;
            error_record(ERR_NETWORK_CONNECT);
        }
    }
    
    // Periodic status update with error handling
    if (net_state == NET_STATE_CONNECTED &&
        current_time - last_status_time >= net_config.report_interval * 1000) {
        if (net_send_status() != ERROR_CODE_SUCCESS) {
            error_record(ERR_NETWORK_STATUS);
        }
        last_status_time = current_time;
    }
}

// Helper function for sending connect packet
static error_code_t send_connect_packet(void) {
    uint8_t packet[3] = {
        NET_MSG_COMMAND,
        0x01,  // Connect command
        calculate_checksum(&packet[1], 1)
    };
    return send_packet(packet, sizeof(packet));
}
