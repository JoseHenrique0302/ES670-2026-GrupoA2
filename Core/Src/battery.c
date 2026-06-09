/**
 * @file battery.c
 * @brief Implements the battery monitoring functions.
 * @details This file contains the functions responsible for
 * initializing the battery monitoring peripherals, calibrating
 * the minimum and maximum charge values, and returning the raw
 * and normalized battery charge.
 * @author Matheus
 * @date 25-Apr-2026
 * @version 1.1
 */

#include "battery.h"
#include "adc.h"
#include "opamp.h"

typedef struct {
  uint16_t usCharge;
  uint16_t usMax;
  uint16_t usMin;
} batteryAttributes_t;

#define BATTERY_MAX_INITIAL_VALUE 4095U
#define BATTERY_MIN_INITIAL_VALUE 2500U
#define BATTERY_PERCENTAGE    100U

batteryAttributes_t xBatteryAttributes;

/**
 * @brief Initializes the battery monitoring peripherals.
 * @details Starts the ADC in DMA mode, enables the operational
 * amplifier, and initializes the battery calibration limits.
 */
void vBatteryInit(void) {
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&xBatteryAttributes.usCharge, 1);
  HAL_OPAMP_Start(&hopamp1);

  xBatteryAttributes.usMax = BATTERY_MAX_INITIAL_VALUE;
  xBatteryAttributes.usMin = BATTERY_MIN_INITIAL_VALUE;
}

/**
 * @brief Calibrates the maximum battery charge value.
 * @details Stores the provided value as the maximum battery charge.
 * @param usValue Value used as the maximum battery charge.
 */
void vBatteryCalibrateMax(uint16_t usValue) {
  xBatteryAttributes.usMax = usValue;
}

/**
 * @brief Calibrates the minimum battery charge value.
 * @details Stores the provided value as the minimum battery charge.
 * @param usValue Value used as the minimum battery charge.
 */
void vBatteryCalibrateMin(uint16_t usValue) {
  xBatteryAttributes.usMin = usValue;
}

/**
 * @brief Returns the raw battery ADC value.
 * @details Returns the current ADC conversion value for the battery.
 * @return Current raw battery ADC value.
 */
uint16_t usBatteryGetRawValue(void) {
  return xBatteryAttributes.usCharge;
}

/**
 * @brief Returns the normalized battery charge.
 * @details Returns the battery charge in percentage using the
 * calibrated minimum and maximum values.
 * @return Battery charge in percentage.
 */
uint16_t usBatteryGetCharge(void) {
  unsigned int uiCharge;
  unsigned int uiRange;

  if (xBatteryAttributes.usCharge <= xBatteryAttributes.usMin) {
    return 0U;
  }

  if (xBatteryAttributes.usCharge >= xBatteryAttributes.usMax) {
    return BATTERY_PERCENTAGE;
  }

  if (xBatteryAttributes.usMax <= xBatteryAttributes.usMin) {
    return 0U;
  }

  uiCharge = (unsigned int)(xBatteryAttributes.usCharge - xBatteryAttributes.usMin);

  uiRange = (unsigned int)(xBatteryAttributes.usMax - xBatteryAttributes.usMin);

  return (uint16_t)(uiCharge * BATTERY_PERCENTAGE / uiRange);
}
