/**
 * @file distanceSensor.c
 * @brief Implements the ultrasonic distance sensor functions.
 * @details This file contains the functions responsible for
 * initializing the trigger and echo timers and calculating the
 * measured distance from the captured echo pulse width.
 * @author Matheus
 * @date 27-Apr-2026
 * @version 1.0
 */

#include "distanceSensor.h"

typedef struct {
  uint16_t usCapture[2];
} distanceSensorAttributes_t;

#define DISTANCESENSOR_CAPTURE_COUNT          2U
#define DISTANCESENSOR_TRIGGER_COMPARE_VALUE  100U
/* Divisor acoplado ao prescaler do timer do ECO (TIM3).
 * TIM3 a 100 kHz (prescaler 1700-1 => 10 us/tick): dist_cm = ticks*10/58 = ticks/5.8.
 * (Antes, TIM3 a 1 MHz/1 us-tick, o divisor era 58.) */
#define DISTANCESENSOR_DISTANCE_DIVISOR       5.8f

static distanceSensorAttributes_t xDistanceSensorAttributes;

/**
 * @brief Initializes the ultrasonic distance sensor.
 * @details This function must be called once during the application
 * initialization, before calling the distance reading function. It
 * starts the PWM signal used as trigger, configures its compare value,
 * starts the echo timer base, and enables the input capture DMA used
 * to store the echo pulse timestamps.
 * @param pTriggerHandle Timer handle used to generate the trigger PWM.
 * @param usTriggerChannel Timer channel used to generate the trigger
 * PWM.
 * @param pEchoHandle Timer handle used to capture the echo pulse.
 * @param usEchoChannel Timer channel used to capture the echo pulse.
 */
void vDistanceSensorInit(TIM_HandleTypeDef *pTriggerHandle,
  uint16_t usTriggerChannel, TIM_HandleTypeDef *pEchoHandle,
  uint16_t usEchoChannel) {

  HAL_TIM_PWM_Start(pTriggerHandle, usTriggerChannel);

  __HAL_TIM_SET_COMPARE(pTriggerHandle, usTriggerChannel, DISTANCESENSOR_TRIGGER_COMPARE_VALUE);

  HAL_TIM_Base_Start(pEchoHandle);

  HAL_TIM_IC_Start_DMA(pEchoHandle, usEchoChannel, (uint32_t *)xDistanceSensorAttributes.usCapture,
	DISTANCESENSOR_CAPTURE_COUNT);
}

/**
 * @brief Returns the measured distance.
 * @details This function should be called after the echo capture values
 * have already been updated by the input capture DMA. The returned
 * value is calculated from the difference between the two captured
 * timestamps.
 * @return Measured distance in centimeters.
 */
float fDistanceSensorGetDistance(void) {
  return (float)(xDistanceSensorAttributes.usCapture[1] -
    xDistanceSensorAttributes.usCapture[0]) / DISTANCESENSOR_DISTANCE_DIVISOR;
}
