/**
 * @file drv_uart_avr_isr.h
 * @brief AVR-specific UART Interrupt Service Routines
 *
 * This file defines the interrupt vectors and handlers for AVR UART ports.
 * It maps the standard AVR UART vectors to our port-specific names and
 * provides the necessary compile-time checks.
 */

#ifndef DRV_UART_AVR_ISR_H
#define DRV_UART_AVR_ISR_H

#include "../../common/drv_uart_types.h"
#include <avr/io.h>

/**
 * @brief UART interrupt vector mappings
 * 
 * Maps the standard AVR UART vectors to our port-specific names.
 * These are used by the ISR implementations to handle UART events.
 * 
 * Note: Your MCU configuration must define these vectors based on
 * the actual hardware UART ports available.
 */
#if defined(USART0_RX_vect)
    #define UART_PORT1_RX_VECT    USART0_RX_vect
    #define UART_PORT1_UDRE_VECT  USART0_UDRE_vect
    #define UART_PORT1_TX_VECT    USART0_TX_vect
#elif defined(USART_RX_vect)
    #define UART_PORT1_RX_VECT    USART_RX_vect
    #define UART_PORT1_UDRE_VECT  USART_UDRE_vect
    #define UART_PORT1_TX_VECT    USART_TX_vect
#endif

#if defined(USART1_RX_vect)
    #define UART_PORT2_RX_VECT    USART1_RX_vect
    #define UART_PORT2_UDRE_VECT  USART1_UDRE_vect
    #define UART_PORT2_TX_VECT    USART1_TX_vect
#endif

#if defined(USART2_RX_vect)
    #define UART_PORT3_RX_VECT    USART2_RX_vect
    #define UART_PORT3_UDRE_VECT  USART2_UDRE_vect
    #define UART_PORT3_TX_VECT    USART2_TX_vect
#endif

/**
 * @brief Compile-time checks for required UART ports
 * 
 * These checks ensure that the required UART ports are available
 * on the target MCU. If a required port is not available, compilation
 * will fail with a helpful error message.
 */
#if defined(UART_REQUIRE_PORT1) && !defined(UART_PORT1_RX_VECT)
    #error "UART Port 1 is required but not available on this MCU"
#endif

#if defined(UART_REQUIRE_PORT2) && !defined(UART_PORT2_RX_VECT)
    #error "UART Port 2 is required but not available on this MCU"
#endif

#if defined(UART_REQUIRE_PORT3) && !defined(UART_PORT3_RX_VECT)
    #error "UART Port 3 is required but not available on this MCU"
#endif

/**
 * @brief UART error masks for status register
 * 
 * These masks are used to check for various UART errors in the
 * status register (UCSRnA).
 */
#define UART_ERR_FRAME     (1 << FE0)  // Frame Error
#define UART_ERR_OVERRUN   (1 << DOR0) // Data Overrun
#define UART_ERR_PARITY    (1 << UPE0) // Parity Error
#define UART_ERR_MASK      (UART_ERR_FRAME | UART_ERR_OVERRUN | UART_ERR_PARITY)

#endif /* DRV_UART_AVR_ISR_H */
