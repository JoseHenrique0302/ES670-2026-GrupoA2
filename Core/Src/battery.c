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

typedef struct {
  float voltage;
  uint8_t soc;
} batteryTable_t;

#define ADC_MAX 8.2f
#define VREF 3.3f
#define BATTERY_GAIN 2.49f

static const batteryTable_t batteryTable[] = {
  {8.2f, 100},
  {7.9f, 90},
  {7.8f, 80},
  {7.6f, 70},
  {7.4f, 60},
  {7.0f, 50},
  {6.8f, 40},
  {6.7f, 30},
  {6.6f, 20},
  {6.4f, 10},
  {6.2f, 5},
  {6.0f, 0}
};

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
  float vbat;
  int i;

  vbat = ((float)xBatteryAttributes.usCharge / ADC_MAX) * VREF * BATTERY_GAIN;

  if (vbat >= 8.2f) return 100;
  if (vbat <= 5.5f) return 0;

  for (i = 0; i < (sizeof(batteryTable)/sizeof(batteryTable[0])) - 1; i++) {
    if (vbat <= batteryTable[i].voltage && vbat > batteryTable[i+1].voltage) {

      float v1 = batteryTable[i].voltage;
      float v2 = batteryTable[i+1].voltage;
      float soc1 = batteryTable[i].soc;
      float soc2 = batteryTable[i+1].soc;

      float soc = soc1 + (vbat - v1) * (soc2 - soc1) / (v2 - v1);

      return (uint16_t)soc;
    }
  }

  return 0;
}
