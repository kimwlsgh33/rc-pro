#include "mcu_power.h"
#include <avr/io.h>
#include <avr/power.h>

error_code_t mcu_power_enable_peripheral(mcu_peripheral_id_t peripheral)
{
    if (peripheral >= MCU_PERIPHERAL_COUNT) {
        return ERR_INVALID_PARAMETER;
    }

    switch (peripheral) {
        case MCU_PERIPHERAL_UART0:
            #if defined(PRR) && defined(PRUSART0)
                PRR &= ~(1 << PRUSART0);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRUSART0)
                PRR0 &= ~(1 << PRUSART0);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_UART1:
            #if defined(PRR) && defined(PRUSART1)
                PRR &= ~(1 << PRUSART1);
                return ERR_SUCCESS;
            #elif defined(PRR1) && defined(PRUSART1)
                PRR1 &= ~(1 << PRUSART1);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_TIMER0:
            #if defined(PRR) && defined(PRTIM0)
                PRR &= ~(1 << PRTIM0);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRTIM0)
                PRR0 &= ~(1 << PRTIM0);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_TIMER1:
            #if defined(PRR) && defined(PRTIM1)
                PRR &= ~(1 << PRTIM1);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRTIM1)
                PRR0 &= ~(1 << PRTIM1);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_TIMER2:
            #if defined(PRR) && defined(PRTIM2)
                PRR &= ~(1 << PRTIM2);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRTIM2)
                PRR0 &= ~(1 << PRTIM2);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_ADC:
            #if defined(PRR) && defined(PRADC)
                PRR &= ~(1 << PRADC);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRADC)
                PRR0 &= ~(1 << PRADC);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_SPI:
            #if defined(PRR) && defined(PRSPI)
                PRR &= ~(1 << PRSPI);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRSPI)
                PRR0 &= ~(1 << PRSPI);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_I2C:
            #if defined(PRR) && defined(PRTWI)
                PRR &= ~(1 << PRTWI);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRTWI)
                PRR0 &= ~(1 << PRTWI);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        default:
            return ERR_NOT_SUPPORTED;
    }
}

error_code_t mcu_power_disable_peripheral(mcu_peripheral_id_t peripheral)
{
    if (peripheral >= MCU_PERIPHERAL_COUNT) {
        return ERR_INVALID_PARAMETER;
    }

    switch (peripheral) {
        case MCU_PERIPHERAL_UART0:
            #if defined(PRR) && defined(PRUSART0)
                PRR |= (1 << PRUSART0);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRUSART0)
                PRR0 |= (1 << PRUSART0);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_UART1:
            #if defined(PRR) && defined(PRUSART1)
                PRR |= (1 << PRUSART1);
                return ERR_SUCCESS;
            #elif defined(PRR1) && defined(PRUSART1)
                PRR1 |= (1 << PRUSART1);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_TIMER0:
            #if defined(PRR) && defined(PRTIM0)
                PRR |= (1 << PRTIM0);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRTIM0)
                PRR0 |= (1 << PRTIM0);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_TIMER1:
            #if defined(PRR) && defined(PRTIM1)
                PRR |= (1 << PRTIM1);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRTIM1)
                PRR0 |= (1 << PRTIM1);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_TIMER2:
            #if defined(PRR) && defined(PRTIM2)
                PRR |= (1 << PRTIM2);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRTIM2)
                PRR0 |= (1 << PRTIM2);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_ADC:
            #if defined(PRR) && defined(PRADC)
                PRR |= (1 << PRADC);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRADC)
                PRR0 |= (1 << PRADC);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_SPI:
            #if defined(PRR) && defined(PRSPI)
                PRR |= (1 << PRSPI);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRSPI)
                PRR0 |= (1 << PRSPI);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        case MCU_PERIPHERAL_I2C:
            #if defined(PRR) && defined(PRTWI)
                PRR |= (1 << PRTWI);
                return ERR_SUCCESS;
            #elif defined(PRR0) && defined(PRTWI)
                PRR0 |= (1 << PRTWI);
                return ERR_SUCCESS;
            #else
                return ERR_NOT_SUPPORTED;
            #endif

        default:
            return ERR_NOT_SUPPORTED;
    }
}
