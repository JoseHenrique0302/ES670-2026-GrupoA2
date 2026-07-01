/**
 * @file      lineSensors.h
 * @brief     Declares the line sensor interface functions and types.
 *
 * This file contains the type definitions and function declarations
 * required to initialize, calibrate, read, and interpolate the line
 * sensor values.
 *
 * @author    Matheus
 * @date      25-Apr-2026
 * @version   2.0
 */

#ifndef LINESENSORS_H
#define LINESENSORS_H

#include <stdint.h>

#define LINESENSORS_SENSOR_COUNT 5U
#define LINESENSORS_ADC_COUNT    5U

typedef enum
{
  LINESENSORS_ADC_1,
  LINESENSORS_ADC_2,
  LINESENSORS_ADC_3,
  LINESENSORS_ADC_4,
  LINESENSORS_ADC_5
} lineSensorsAdcEnum_t;

typedef enum
{
  LINESENSORS_RANK_1,
  LINESENSORS_RANK_2,
  LINESENSORS_RANK_3,
  LINESENSORS_RANK_4,
  LINESENSORS_RANK_5
} lineSensorsRankEnum_t;

typedef enum
{
  LINESENSORS_SENSOR_LEFT,
  LINESENSORS_SENSOR_CENTERLEFT,
  LINESENSORS_SENSOR_CENTER,
  LINESENSORS_SENSOR_CENTERRIGHT,
  LINESENSORS_SENSOR_RIGHT
} lineSensorsEnum_t;

/**
 * @brief Initializes the line sensors and starts ADC DMA acquisition.
 *
 * Configures the ADC and rank used by each line sensor, starts only the
 * ADC DMA acquisitions that are required, associates each sensor with
 * the correct DMA buffer position, configures the interpolation weights,
 * and resets the calibration values.
 *
 * @param xLeftAdc ADC used by the left sensor.
 * @param xLeftRank Rank used by the left sensor.
 * @param xCenterLeftAdc ADC used by the center-left sensor.
 * @param xCenterLeftRank Rank used by the center-left sensor.
 * @param xCenterAdc ADC used by the center sensor.
 * @param xCenterRank Rank used by the center sensor.
 * @param xCenterRightAdc ADC used by the center-right sensor.
 * @param xCenterRightRank Rank used by the center-right sensor.
 * @param xRightAdc ADC used by the right sensor.
 * @param xRightRank Rank used by the right sensor.
 */
void vLineSensors_v2_Init(lineSensorsAdcEnum_t xLeftAdc,
                      lineSensorsRankEnum_t xLeftRank,
                      lineSensorsAdcEnum_t xCenterLeftAdc,
                      lineSensorsRankEnum_t xCenterLeftRank,
                      lineSensorsAdcEnum_t xCenterAdc,
                      lineSensorsRankEnum_t xCenterRank,
                      lineSensorsAdcEnum_t xCenterRightAdc,
                      lineSensorsRankEnum_t xCenterRightRank,
                      lineSensorsAdcEnum_t xRightAdc,
                      lineSensorsRankEnum_t xRightRank);

/**
 * @brief Resets the calibration limits of all line sensors.
 */
void vLineSensors_v2_ResetCalibration(void);

/**
 * @brief Updates the calibration limits using the current ADC readings.
 */
void vLineSensors_v2_UpdateCalibration(void);

/**
 * @brief Returns the normalized value of a selected line sensor.
 *
 * @param xSensor Selected sensor identifier.
 * @return Normalized sensor value in the range from 0.0 to 1.0.
 */
float fLineSensors_v2_GetSensorValue(lineSensorsEnum_t xSensor);

/**
 * @brief Returns the RAW (uncalibrated) ADC value of a selected line sensor.
 *
 * Util p/ depurar no debugger: mostra o sensor respondendo (mudando) mesmo
 * quando a calibracao (min/max) estiver ruim e o valor normalizado travado.
 *
 * @param xSensor Selected sensor identifier.
 * @return Raw ADC value (0..4095).
 */
uint16_t usLineSensors_v2_GetRawValue(lineSensorsEnum_t xSensor);

/**
 * @brief Sets the interpolation weights used by the line sensors.
 */
void vLineSensors_v2_SetInterpolationWeigths(float fLeftWeigth,
  float fCenterLeftWeigth,
  float fCenterWeigth,
  float fCenterRightWeigth,
  float fRightWeigth);

/**
 * @brief Returns the interpolated value calculated from all sensors.
 *
 * @return Interpolated line position based on the configured weights.
 */
float fLineSensors_v2_GetInterpolatedValue(void);

#endif // LINESENSORS_H
