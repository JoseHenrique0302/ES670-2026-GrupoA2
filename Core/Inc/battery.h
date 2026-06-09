/**
 * @file battery.h
 * @brief Declares the battery monitoring interface.
 * @details This file contains the function declarations required
 * to initialize, calibrate, and read the battery charge.
 * @author Matheus
 * @date 25-Apr-2026
 * @version 1.1
 */

#ifndef BATTERY_H
#define BATTERY_H

#include "main.h"

/**
 * @brief Initializes the battery monitoring peripherals.
 * @details Starts the ADC in DMA mode, enables the operational
 * amplifier, and initializes the battery calibration limits.
 */
void vBatteryInit(void);

/**
 * @brief Calibrates the maximum battery charge value.
 * @details Stores the provided value as the maximum battery charge.
 * @param usValue Value used as the maximum battery charge.
 */
void vBatteryCalibrateMax(uint16_t usValue);

/**
 * @brief Calibrates the minimum battery charge value.
 * @details Stores the provided value as the minimum battery charge.
 * @param usValue Value used as the minimum battery charge.
 */
void vBatteryCalibrateMin(uint16_t usValue);

/**
 * @brief Returns the raw battery ADC value.
 * @details Returns the current ADC conversion value for the battery.
 * @return Current raw battery ADC value.
 */
uint16_t usBatteryGetRawValue(void);

/**
 * @brief Returns the normalized battery charge.
 * @details Returns the battery charge in percentage using the
 * calibrated minimum and maximum values.
 * @return Battery charge in percentage.
 */
uint16_t usBatteryGetCharge(void);

#endif // BATTERY_H
