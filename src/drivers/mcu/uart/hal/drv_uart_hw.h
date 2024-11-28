#ifndef DRV_UART_HW_H
#define DRV_UART_HW_H

#include "../common/drv_uart_types.h"
#include <avr/io.h>

/**
 * @file drv_uart_hw.h
 * @brief UART Hardware Access Layer
 *
 * This file contains all hardware-specific definitions and access functions
 * for the UART peripheral. It provides a complete hardware abstraction layer
 * that isolates all direct register access to this single file.
 *
 * Key Features:
 * - Register bit definitions
 * - Configuration masks
 * - Validation macros
 * - Register access functions
 * - Error status definitions
 */

/* Register Bit Definitions */

/**
 * @brief UART Status Register A (UCSRnA) bit definitions
 * 
 * These bits reflect the current status of the UART hardware:
 * - RXC: Set when receive is complete
 * - TXC: Set when transmit is complete
 * - UDRE: Set when data register is empty
 * - FE: Set on frame error
 * - DOR: Set on data overrun
 * - UPE: Set on parity error
 * - U2X: Double transmission speed
 * - MPCM: Multi-processor communication mode
 */
#define UART_UCSRA_RXC     (1 << RXC0)   // Receive Complete
#define UART_UCSRA_TXC     (1 << TXC0)   // Transmit Complete
#define UART_UCSRA_UDRE    (1 << UDRE0)  // Data Register Empty
#define UART_UCSRA_FE      (1 << FE0)    // Frame Error
#define UART_UCSRA_DOR     (1 << DOR0)   // Data OverRun
#define UART_UCSRA_UPE     (1 << UPE0)   // Parity Error
#define UART_UCSRA_U2X     (1 << U2X0)   // Double Speed Operation
#define UART_UCSRA_MPCM    (1 << MPCM0)  // Multi-processor Communication Mode

/**
 * @brief UART Control Register B (UCSRnB) bit definitions
 * 
 * These bits control the UART operation:
 * - RXCIE: Enable RX Complete Interrupt
 * - TXCIE: Enable TX Complete Interrupt
 * - UDRIE: Enable Data Register Empty Interrupt
 * - RXEN: Enable Receiver
 * - TXEN: Enable Transmitter
 * - UCSZ2: Character Size bit 2
 * - RXB8/TXB8: 9th data bit for 9-bit operation
 */
#define UART_UCSRB_RXCIE   (1 << RXCIE0) // RX Complete Interrupt Enable
#define UART_UCSRB_TXCIE   (1 << TXCIE0) // TX Complete Interrupt Enable
#define UART_UCSRB_UDRIE   (1 << UDRIE0) // USART Data Register Empty Interrupt Enable
#define UART_UCSRB_RXEN    (1 << RXEN0)  // Receiver Enable
#define UART_UCSRB_TXEN    (1 << TXEN0)  // Transmitter Enable
#define UART_UCSRB_UCSZ2   (1 << UCSZ02) // Character Size bit 2
#define UART_UCSRB_RXB8    (1 << RXB80)  // Receive Data Bit 8
#define UART_UCSRB_TXB8    (1 << TXB80)  // Transmit Data Bit 8

/**
 * @brief UART Control Register C (UCSRnC) bit definitions
 * 
 * These bits configure the UART frame format:
 * - UMSEL: USART Mode Select (Async/Sync/SPI)
 * - UPM: Parity Mode
 * - USBS: Stop Bit Select
 * - UCSZ: Character Size
 * - UCPOL: Clock Polarity (for sync mode)
 */
#define UART_UCSRC_UMSEL1  (1 << UMSEL01) // USART Mode Select bit 1
#define UART_UCSRC_UMSEL0  (1 << UMSEL00) // USART Mode Select bit 0
#define UART_UCSRC_UPM1    (1 << UPM01)   // Parity Mode bit 1
#define UART_UCSRC_UPM0    (1 << UPM00)   // Parity Mode bit 0
#define UART_UCSRC_USBS    (1 << USBS0)   // Stop Bit Select
#define UART_UCSRC_UCSZ1   (1 << UCSZ01)  // Character Size bit 1
#define UART_UCSRC_UCSZ0   (1 << UCSZ00)  // Character Size bit 0
#define UART_UCSRC_UCPOL   (1 << UCPOL0)  // Clock Polarity

/**
 * @brief UART Mode Selection masks
 * 
 * Pre-defined masks for configuring UART operation mode:
 * - ASYNC: Standard asynchronous UART
 * - SYNC: Synchronous UART
 * - MASTER_SPI: SPI Master mode
 */
#define UART_MODE_ASYNC    0x00
#define UART_MODE_SYNC     UART_UCSRC_UMSEL0
#define UART_MODE_MASTER_SPI (UART_UCSRC_UMSEL1 | UART_UCSRC_UMSEL0)

/**
 * @brief UART Parity Selection masks
 * 
 * Pre-defined masks for configuring parity:
 * - NONE: No parity
 * - EVEN: Even parity
 * - ODD: Odd parity
 */
#define UART_PARITY_NONE   0x00
#define UART_PARITY_EVEN   UART_UCSRC_UPM1
#define UART_PARITY_ODD    (UART_UCSRC_UPM1 | UART_UCSRC_UPM0)

/**
 * @brief UART Stop Bit Selection masks
 */
#define UART_STOP_1BIT     0x00
#define UART_STOP_2BIT     UART_UCSRC_USBS

/**
 * @brief UART Character Size Selection masks
 * 
 * Pre-defined masks for configuring character size:
 * Note: 9-bit mode requires additional UCSZ2 bit in UCSRnB
 */
#define UART_CHARSIZE_5BIT 0x00
#define UART_CHARSIZE_6BIT UART_UCSRC_UCSZ0
#define UART_CHARSIZE_7BIT UART_UCSRC_UCSZ1
#define UART_CHARSIZE_8BIT (UART_UCSRC_UCSZ1 | UART_UCSRC_UCSZ0)
#define UART_CHARSIZE_9BIT (UART_UCSRC_UCSZ1 | UART_UCSRC_UCSZ0 | UART_UCSRB_UCSZ2)

/**
 * @brief UART Error Status mask
 * 
 * Combined mask for all error bits in UCSRnA:
 * - Frame Error
 * - Data Overrun
 * - Parity Error
 */
#define UART_ERR_MASK    (UART_UCSRA_FE | UART_UCSRA_DOR | UART_UCSRA_UPE)

/**
 * @brief UART Configuration validation macros
 * 
 * These macros validate configuration parameters:
 * - Character size must be 5-9 bits
 * - Stop bits must be 1 or 2
 * - Parity must be 0 (none), 1 (even), or 2 (odd)
 * - Baud rate must be within achievable range for given F_CPU
 */
#define UART_IS_VALID_CHARSIZE(size) ((size) >= 5 && (size) <= 9)
#define UART_IS_VALID_STOPBITS(stop) ((stop) == 1 || (stop) == 2)
#define UART_IS_VALID_PARITY(par)    ((par) <= 2)

/**
 * @brief UART Baud rate validation
 * 
 * Calculates valid baud rate range based on F_CPU:
 * - Minimum: Accounts for maximum divisor in non-U2X mode
 * - Maximum: Accounts for minimum divisor in U2X mode
 */
#define UART_MIN_BAUD  (F_CPU/16/65535)
#define UART_MAX_BAUD  (F_CPU/8)
#define UART_IS_VALID_BAUD(baud) ((baud) >= UART_MIN_BAUD && (baud) <= UART_MAX_BAUD)

/* Interrupt Vector Definitions */
#if defined(USART0_RX_vect)
    #define UART_PORT1_RX_VECT USART0_RX_vect
    #define UART_PORT1_TX_VECT USART0_TX_vect
#elif defined(USART_RX_vect)
    #define UART_PORT1_RX_VECT USART_RX_vect
    #define UART_PORT1_TX_VECT USART_TX_vect
#endif

#if defined(USART1_RX_vect)
    #define UART_PORT2_RX_VECT USART1_RX_vect
    #define UART_PORT2_TX_VECT USART1_TX_vect
#endif

#if defined(USART2_RX_vect)
    #define UART_PORT3_RX_VECT USART2_RX_vect
    #define UART_PORT3_TX_VECT USART2_TX_vect
#endif

/* Register Access Interface */
/**
 * @brief Global array of UART registers for direct HAL access
 * This array is initialized at startup and provides quick access
 * to the registers for the HAL implementation
 */
extern const uart_registers_t uart_registers[UART_PORT_MAX];

/**
 * @brief Initialize UART register mapping
 * Must be called before any UART operations
 */
void uart_hw_init(void);

/**
 * @brief Get hardware registers for specified UART port
 * 
 * This function maps the logical UART port to its hardware registers.
 * It provides access to all necessary control and data registers for
 * the specified UART port.
 * 
 * Note: This function is primarily used by ISRs and should be kept inline
 * for performance. For regular HAL operations, use the uart_registers array.
 * 
 * @param port UART port number
 * @return Register structure for the specified port
 */
static inline uart_registers_t get_uart_registers(uart_port_t port) {
    uart_registers_t regs = {0};
    
    switch (port) {
#if defined(UDR0)
        case UART_PORT_1:
            regs.UDRn = &UDR0;
            regs.UCSRnA = &UCSR0A;
            regs.UCSRnB = &UCSR0B;
            regs.UCSRnC = &UCSR0C;
            regs.UBRRnH = &UBRR0H;
            regs.UBRRnL = &UBRR0L;
            regs.RXCIEn = RXCIE0;
            regs.TXCIEn = TXCIE0;
            regs.UDRIEn = UDRIE0;
            break;
#endif
#if defined(UDR1)
        case UART_PORT_2:
            regs.UDRn = &UDR1;
            regs.UCSRnA = &UCSR1A;
            regs.UCSRnB = &UCSR1B;
            regs.UCSRnC = &UCSR1C;
            regs.UBRRnH = &UBRR1H;
            regs.UBRRnL = &UBRR1L;
            regs.RXCIEn = RXCIE1;
            regs.TXCIEn = TXCIE1;
            regs.UDRIEn = UDRIE1;
            break;
#endif
#if defined(UDR2)
        case UART_PORT_3:
            regs.UDRn = &UDR2;
            regs.UCSRnA = &UCSR2A;
            regs.UCSRnB = &UCSR2B;
            regs.UCSRnC = &UCSR2C;
            regs.UBRRnH = &UBRR2H;
            regs.UBRRnL = &UBRR2L;
            regs.RXCIEn = RXCIE2;
            regs.TXCIEn = TXCIE2;
            regs.UDRIEn = UDRIE2;
            break;
#endif
        default:
            // Return zeroed structure for invalid ports
            break;
    }
    return regs;
}

#endif /* DRV_UART_HW_H */
