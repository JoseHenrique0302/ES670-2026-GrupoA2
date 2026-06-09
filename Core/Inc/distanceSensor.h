/**
 * @file distanceSensor.h
 * @brief Declares the ultrasonic distance sensor interface.
 * @details This file contains the function declarations required to
 * initialize the ultrasonic distance sensor and read the measured
 * distance.
 * @author Matheus
 * @date 27-Apr-2026
 * @version 1.0
 */

#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include "tim.h"

/**
 * @brief Initializes the ultrasonic distance sensor.
 * @details This function must be called once during the application
 * initialization, before calling the distance reading function. It
 * configures the timers used to generate the trigger signal and capture
 * the echo pulse.
 * @param pTriggerHandle Timer handle used to generate the trigger PWM.
 * @param usTriggerChannel Timer channel used to generate the trigger
 * PWM.
 * @param pEchoHandle Timer handle used to capture the echo pulse.
 * @param usEchoChannel Timer channel used to capture the echo pulse.
 */
void vDistanceSensorInit(TIM_HandleTypeDef *pTriggerHandle,
  uint16_t usTriggerChannel, TIM_HandleTypeDef *pEchoHandle,
  uint16_t usEchoChannel);

/**
 * @brief Returns the measured distance.
 * @details This function should be called only after the echo capture
 * values have already been updated by the input capture DMA, such as in
 * the main loop or in a periodic task that runs after the measurement
 * has completed.
 * @return Measured distance in centimeters.
 */
float fDistanceSensorGetDistance(void);

// Adicione no final, antes do #endif

#endif // DISTANCESENSOR_H
