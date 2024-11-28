#ifndef MCU_POWER_H
#define MCU_POWER_H

#include <stdint.h>
#include "mcu_types.h"
#include "../../../utils/common/error_codes.h"

/**
 * @brief Enable power to a specific peripheral
 * @param peripheral The peripheral to enable
 * @return error_code_t ERR_SUCCESS if successful, ERR_INVALID_PARAMETER if peripheral is invalid,
 *         or ERR_NOT_SUPPORTED if the peripheral is not supported on this MCU
 */
error_code_t mcu_power_enable_peripheral(mcu_peripheral_id_t peripheral);

/**
 * @brief Disable power to a specific peripheral
 * @param peripheral The peripheral to disable
 * @return error_code_t ERR_SUCCESS if successful, ERR_INVALID_PARAMETER if peripheral is invalid,
 *         or ERR_NOT_SUPPORTED if the peripheral is not supported on this MCU
 */
error_code_t mcu_power_disable_peripheral(mcu_peripheral_id_t peripheral);

#endif // MCU_POWER_H
