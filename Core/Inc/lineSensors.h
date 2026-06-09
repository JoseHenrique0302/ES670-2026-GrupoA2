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
 * @version   1.0
 */

#ifndef LINESENSORS_H
#define LINESENSORS_H

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
 * Configures the ADC channels used by the line sensors, associates
 * each sensor with its corresponding buffer, sets the interpolation
 * weights, and resets the calibration values.
 */
void vLineSensorsInit(void);

/**
 * @brief Resets the calibration limits of all line sensors.
 *
 * Initializes the minimum and maximum values used during the
 * calibration process.
 */
void vLineSensorsResetCalibration(void);

/**
 * @brief Updates the calibration limits using the current ADC readings.
 *
 * Compares the current sensor readings with the stored calibration
 * limits and updates the minimum and maximum values when necessary.
 */
void vLineSensorsUpdateCalibration(void);

/**
 * @brief Returns the normalized value of a selected line sensor.
 *
 * @param xSensor Selected sensor identifier.
 * @return Normalized sensor value in the range from 0.0 to 1.0.
 */
float fLineSensorsGetSensorValue(lineSensorsEnum_t xSensor);

/**
 * @brief Sets the interpolation weights used by the line sensors.
 *
 * @param fLeftWeigth Weight assigned to the left sensor.
 * @param fCenterLeftWeigth Weight assigned to the center-left sensor.
 * @param fCenterWeigth Weight assigned to the center sensor.
 * @param fCenterRightWeigth Weight assigned to the center-right sensor.
 * @param fRightWeigth Weight assigned to the right sensor.
 */
void vLineSensorsSetInterpolationWeigths(float fLeftWeigth,
  float fCenterLeftWeigth,
  float fCenterWeigth,
  float fCenterRightWeigth,
  float fRightWeigth);

/**
 * @brief Returns the interpolated value calculated from all sensors.
 *
 * @return Interpolated line position based on the configured weights.
 */
float fLineSensorsGetInterpolatedValue(void);

float fLineSensorGetRawValue(lineSensorsEnum_t xSensor);
float fLineSensorGetMax(lineSensorsEnum_t xSensor);
float fLineSensorGetMin(lineSensorsEnum_t xSensor);

#endif // LINESENSORS_H
