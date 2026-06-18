/**
 * @file      lineSensors.c
 * @brief     Implements the line sensor acquisition and interpolation.
 *
 * Versão 2 (driver do professor) com mapeamento sensor->ADC/rank
 * configurável. Único ajuste em relação ao arquivo original do professor:
 * o init usa os handles ADC REAIS do projeto (hadc1/2/3/5, definidos em
 * adc.c) em vez de handles locais só com .Instance — caso contrário o
 * HAL_ADC_Start_DMA acessaria DMA_Handle inválido e travaria no boot.
 *
 * @author    Matheus
 * @date      25-Apr-2026
 * @version   2.0
 */

#include "lineSensors.h"
#include "main.h"
#include "adc.h"

#define LINESENSORS_INITIAL_MAX  1500U
#define LINESENSORS_INITIAL_MIN  1000U

typedef struct
{
  lineSensorsAdcEnum_t xAdc;
  lineSensorsRankEnum_t xRank;
} lineSensorsConfig_t;

typedef struct
{
  uint16_t usBufferAdc1[LINESENSORS_SENSOR_COUNT];
  uint16_t usBufferAdc2[LINESENSORS_SENSOR_COUNT];
  uint16_t usBufferAdc3[LINESENSORS_SENSOR_COUNT];
  uint16_t usBufferAdc4[LINESENSORS_SENSOR_COUNT];
  uint16_t usBufferAdc5[LINESENSORS_SENSOR_COUNT];
  uint16_t *pBuffer[LINESENSORS_SENSOR_COUNT];
  uint16_t usMax[LINESENSORS_SENSOR_COUNT];
  uint16_t usMin[LINESENSORS_SENSOR_COUNT];
  float fWeigths[LINESENSORS_SENSOR_COUNT];
} lineSensorsAttributes_t;

static lineSensorsAttributes_t xLineSensorsAttributes;

void vLineSensors_v2_Init(lineSensorsAdcEnum_t xLeftAdc,
                      lineSensorsRankEnum_t xLeftRank,
                      lineSensorsAdcEnum_t xCenterLeftAdc,
                      lineSensorsRankEnum_t xCenterLeftRank,
                      lineSensorsAdcEnum_t xCenterAdc,
                      lineSensorsRankEnum_t xCenterRank,
                      lineSensorsAdcEnum_t xCenterRightAdc,
                      lineSensorsRankEnum_t xCenterRightRank,
                      lineSensorsAdcEnum_t xRightAdc,
                      lineSensorsRankEnum_t xRightRank)
{
  lineSensorsConfig_t xConfig[LINESENSORS_SENSOR_COUNT];
  unsigned char ucAdcCount[LINESENSORS_ADC_COUNT] = {0U};
  unsigned char ucIndex;
  uint16_t *pAdcBuffer[LINESENSORS_ADC_COUNT];
  /* AJUSTE: ponteiros para os handles ADC REAIS (adc.c). ADC4 nao existe
   * neste projeto -> fica NULL e nunca e iniciado (o mapeamento nao o usa). */
  ADC_HandleTypeDef *pAdcHandle[LINESENSORS_ADC_COUNT];

  pAdcBuffer[LINESENSORS_ADC_1] = xLineSensorsAttributes.usBufferAdc1;
  pAdcBuffer[LINESENSORS_ADC_2] = xLineSensorsAttributes.usBufferAdc2;
  pAdcBuffer[LINESENSORS_ADC_3] = xLineSensorsAttributes.usBufferAdc3;
  pAdcBuffer[LINESENSORS_ADC_4] = xLineSensorsAttributes.usBufferAdc4;
  pAdcBuffer[LINESENSORS_ADC_5] = xLineSensorsAttributes.usBufferAdc5;

  pAdcHandle[LINESENSORS_ADC_1] = &hadc1;
  pAdcHandle[LINESENSORS_ADC_2] = &hadc2;
  pAdcHandle[LINESENSORS_ADC_3] = &hadc3;
  pAdcHandle[LINESENSORS_ADC_4] = NULL;   /* ADC4 nao configurado no projeto */
  pAdcHandle[LINESENSORS_ADC_5] = &hadc5;

  xConfig[LINESENSORS_SENSOR_LEFT].xAdc = xLeftAdc;
  xConfig[LINESENSORS_SENSOR_LEFT].xRank = xLeftRank;

  xConfig[LINESENSORS_SENSOR_CENTERLEFT].xAdc = xCenterLeftAdc;
  xConfig[LINESENSORS_SENSOR_CENTERLEFT].xRank = xCenterLeftRank;

  xConfig[LINESENSORS_SENSOR_CENTER].xAdc = xCenterAdc;
  xConfig[LINESENSORS_SENSOR_CENTER].xRank = xCenterRank;

  xConfig[LINESENSORS_SENSOR_CENTERRIGHT].xAdc = xCenterRightAdc;
  xConfig[LINESENSORS_SENSOR_CENTERRIGHT].xRank = xCenterRightRank;

  xConfig[LINESENSORS_SENSOR_RIGHT].xAdc = xRightAdc;
  xConfig[LINESENSORS_SENSOR_RIGHT].xRank = xRightRank;

  for (ucIndex = 0U; ucIndex < LINESENSORS_SENSOR_COUNT; ucIndex++)
  {
    ucAdcCount[xConfig[ucIndex].xAdc]++;
  }

  for (ucIndex = 0U; ucIndex < LINESENSORS_SENSOR_COUNT; ucIndex++)
  {
    xLineSensorsAttributes.pBuffer[ucIndex] =
      &pAdcBuffer[xConfig[ucIndex].xAdc][xConfig[ucIndex].xRank];
  }

  for (ucIndex = 0U; ucIndex < LINESENSORS_ADC_COUNT; ucIndex++)
  {
    if ((0U != ucAdcCount[ucIndex]) && (NULL != pAdcHandle[ucIndex]))
    {
      HAL_ADC_Start_DMA(pAdcHandle[ucIndex],
                        (uint32_t *)pAdcBuffer[ucIndex],
                        ucAdcCount[ucIndex]);
    }
  }

  vLineSensors_v2_SetInterpolationWeigths(-0.5f, -1.0f, 0.0f, 1.0f, 0.5f);
  vLineSensors_v2_ResetCalibration();
}

void vLineSensors_v2_ResetCalibration(void)
{
  unsigned int uiIndex;

  for (uiIndex = 0U; uiIndex < LINESENSORS_SENSOR_COUNT; uiIndex++)
  {
    xLineSensorsAttributes.usMax[uiIndex] = LINESENSORS_INITIAL_MAX;
    xLineSensorsAttributes.usMin[uiIndex] = LINESENSORS_INITIAL_MIN;
  }
}

void vLineSensors_v2_UpdateCalibration(void)
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

float fLineSensors_v2_GetSensorValue(lineSensorsEnum_t xSensor)
{
  float fRange;
  float fValue;

  fRange = (float)(xLineSensorsAttributes.usMax[xSensor] -
    xLineSensorsAttributes.usMin[xSensor]);

  if (0.0f == fRange)
  {
    return 0.0f;   /* evita divisao por zero se max==min */
  }

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

void vLineSensors_v2_SetInterpolationWeigths(float fLeftWeigth,
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

float fLineSensors_v2_GetInterpolatedValue(void)
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

    fValue += fLineSensors_v2_GetSensorValue((lineSensorsEnum_t)uiIndex) *
      xLineSensorsAttributes.fWeigths[uiIndex];
  }

  return (0.0f < fValue) ? (fValue / fPosMax) : (fValue / fNegMax);
}
