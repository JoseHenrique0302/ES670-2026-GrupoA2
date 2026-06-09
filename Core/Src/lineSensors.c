/**
 * @file      lineSensors.c
 * @brief     Implements the line sensor acquisition and interpolation.
 *
 * This file contains the functions required to initialize the line
 * sensors, update their calibration limits, read normalized values,
 * and calculate the interpolated line position.
 *
 * @author    Matheus
 * @date      25-Apr-2026
 * @version   1.0
 */

#ifndef LINESENSORS_C
#define LINESENSORS_C

#include "lineSensors.h"
#include "main.h"
#include "adc.h"

typedef struct
{
  uint16_t usBuffer124[3];
  uint16_t usBuffer3;
  uint16_t usBuffer5;
  uint16_t *pBuffer[5];
  uint16_t usMax[5];
  uint16_t usMin[5];
  float fWeigths[5];
} lineSensorsAttributes_t;

#define LINESENSORS_SENSOR_COUNT 5U
#define LINESENSORS_INITIAL_MAX  2000U
#define LINESENSORS_INITIAL_MIN  1500U

lineSensorsAttributes_t xLineSensorsAttributes;

/**
 * @brief Initializes the line sensors and starts ADC DMA acquisition.
 *
 * Starts the ADC peripherals in DMA mode, associates each sensor with
 * its respective buffer, configures the interpolation weights, and
 * resets the calibration values.
 */
void vLineSensorsInit(void)
{
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)xLineSensorsAttributes.usBuffer124, 3);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&xLineSensorsAttributes.usBuffer3, 1);
  HAL_ADC_Start_DMA(&hadc5, (uint32_t *)&xLineSensorsAttributes.usBuffer5, 1);

  xLineSensorsAttributes.pBuffer[0] = &xLineSensorsAttributes.usBuffer124[0];
  xLineSensorsAttributes.pBuffer[1] = &xLineSensorsAttributes.usBuffer124[1];
  xLineSensorsAttributes.pBuffer[2] = &xLineSensorsAttributes.usBuffer3;
  xLineSensorsAttributes.pBuffer[3] = &xLineSensorsAttributes.usBuffer124[2];
  xLineSensorsAttributes.pBuffer[4] = &xLineSensorsAttributes.usBuffer5;

  vLineSensorsSetInterpolationWeigths(-0.5f, -1.0f, 0.0f, 1.0f, 0.5f);
  vLineSensorsResetCalibration();
}

/**
 * @brief Resets the calibration limits of all line sensors.
 *
 * Initializes the maximum and minimum values used during the
 * calibration process.
 */
void vLineSensorsResetCalibration(void)
{
  unsigned int uiIndex;

  for (uiIndex = 0U; uiIndex < LINESENSORS_SENSOR_COUNT; uiIndex++)
  {
    xLineSensorsAttributes.usMax[uiIndex] = LINESENSORS_INITIAL_MAX;
    xLineSensorsAttributes.usMin[uiIndex] = LINESENSORS_INITIAL_MIN;
  }
}

/**
 * @brief Updates the calibration limits using the current ADC readings.
 *
 * Compares the current sensor readings with the stored calibration
 * limits and updates the maximum and minimum values when necessary.
 */
void vLineSensorsUpdateCalibration(void)
{
  unsigned int uiIndex;

  for (uiIndex = 0U; uiIndex < LINESENSORS_SENSOR_COUNT; uiIndex++)
  {
    if (*xLineSensorsAttributes.pBuffer[uiIndex] >
        xLineSensorsAttributes.usMax[uiIndex])
    {
      xLineSensorsAttributes.usMax[uiIndex] =
        *xLineSensorsAttributes.pBuffer[uiIndex];
    }

    if (*xLineSensorsAttributes.pBuffer[uiIndex] <
        xLineSensorsAttributes.usMin[uiIndex])
    {
      xLineSensorsAttributes.usMin[uiIndex] =
        *xLineSensorsAttributes.pBuffer[uiIndex];
    }
  }
}

/**
 * @brief Returns the normalized value of a selected line sensor.
 *
 * @param xSensor Selected sensor identifier.
 * @return Normalized sensor value in the range from 0.0 to 1.0.
 */
float fLineSensorsGetSensorValue(lineSensorsEnum_t xSensor)
{
  float fRange;
  float fValue;

  fRange = (float)(xLineSensorsAttributes.usMax[xSensor] -
    xLineSensorsAttributes.usMin[xSensor]);
  fValue = ((float)(*xLineSensorsAttributes.pBuffer[xSensor]) -
    (float)xLineSensorsAttributes.usMin[xSensor]) / fRange;

  if (1.0f < fValue)
  {
    fValue = 1.0f;
  }

  if (0.0f > fValue)
  {
    fValue = 0.0f;
  }

  return fValue;
}

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
  float fRightWeigth)
{
  xLineSensorsAttributes.fWeigths[0] = fLeftWeigth;
  xLineSensorsAttributes.fWeigths[1] = fCenterLeftWeigth;
  xLineSensorsAttributes.fWeigths[2] = fCenterWeigth;
  xLineSensorsAttributes.fWeigths[3] = fCenterRightWeigth;
  xLineSensorsAttributes.fWeigths[4] = fRightWeigth;
}

/**
 * @brief Returns the interpolated value calculated from all sensors.
 *
 * @return Interpolated line position based on the configured weights.
 */
float fLineSensorsGetInterpolatedValue(void)
{
  float fNegMax;
  float fPosMax;
  float fValue;
  unsigned int uiIndex;

  fNegMax = 0.0f;
  fPosMax = 0.0f;
  fValue = 0.0f;

  for (uiIndex = 0U; uiIndex < LINESENSORS_SENSOR_COUNT; uiIndex++)
  {
    if (0.0f < xLineSensorsAttributes.fWeigths[uiIndex])
    {
      fPosMax += xLineSensorsAttributes.fWeigths[uiIndex];
    }
    else
    {
      fNegMax -= xLineSensorsAttributes.fWeigths[uiIndex];
    }

    fValue += fLineSensorsGetSensorValue((lineSensorsEnum_t)uiIndex) *
      xLineSensorsAttributes.fWeigths[uiIndex];
  }

  return (0.0f < fValue) ? (fValue / fPosMax) : (fValue / fNegMax);
}

// Funções de acesso público para os valores crus, máximos e mínimos dos sensores
float fLineSensorGetRawValue(lineSensorsEnum_t xSensor)
{
    if ((uint8_t)xSensor < LINESENSORS_SENSOR_COUNT)
    {
        return *xLineSensorsAttributes.pBuffer[(uint8_t)xSensor];
    }
    return 0;
}

float fLineSensorGetMax(lineSensorsEnum_t xSensor)
{
    if ((uint8_t)xSensor < LINESENSORS_SENSOR_COUNT)
    {
        return xLineSensorsAttributes.usMax[(uint8_t)xSensor];
    }
    return 0;
}

float fLineSensorGetMin(lineSensorsEnum_t xSensor)
{
    if ((uint8_t)xSensor < LINESENSORS_SENSOR_COUNT)
    {
        return xLineSensorsAttributes.usMin[(uint8_t)xSensor];
    }
    return 0;
}

#endif // LINESENSORS_C
