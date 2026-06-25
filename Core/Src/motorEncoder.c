/**
 * @file motorEncoder.c
 * @brief Implements the motor control and encoder reading functions.
 *
 * This file contains the functions required to control the motors,
 * configure the encoder timers, and calculate the measured motor
 * velocities.
 *
 * @author rodri
 * @date 19-Oct-2025
 * @version 1.1
 */

#include "motorEncoder.h"

typedef struct {
  GPIO_TypeDef *pIn1Port;
  uint16_t usIn1Pin;
  GPIO_TypeDef *pIn2Port;
  uint16_t usIn2Pin;
  GPIO_TypeDef *pIn3Port;
  uint16_t usIn3Pin;
  GPIO_TypeDef *pIn4Port;
  uint16_t usIn4Pin;
  TIM_HandleTypeDef *pRightMotorTimerHandle;
  unsigned int uiRightMotorTimerChannel;
  TIM_HandleTypeDef *pLeftMotorTimerHandle;
  unsigned int uiLeftMotorTimerChannel;
  TIM_HandleTypeDef *pRightEncoderTimerHandle;
  unsigned int uiRightEncoderTimerChannel;
  TIM_HandleTypeDef *pLeftEncoderTimerHandle;
  unsigned int uiLeftEncoderTimerChannel;
} motorEncoderPeripherals_t;

#define MOTORENCODER_PWM_COMPARE_MAX 999.0f

static motorEncoderPeripherals_t xMotorEncoderPeripherals;
static motorEncoderAttributes_t xMotorEncoderAttributes;

/**
 * @brief Sets the direction pins of one motor.
 *
 * This internal function is called by vMotorEncoderControlMotor before
 * updating the PWM duty cycle of the selected motor.
 *
 * @param xMotor Selected motor.
 * @param xDirection Direction applied to the selected motor.
 */
static void vMotorEncoderSetDirection(motorEncoderMotor_t xMotor,
  motorEncoderDirection_t xDirection)
{
  switch (xMotor) {
    case MOTORENCODER_MOTOR_RIGHT:
      switch (xDirection) {
        case MOTORENCODER_DIRECTION_FORWARD:
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn1Port,
            xMotorEncoderPeripherals.usIn1Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn2Port,
            xMotorEncoderPeripherals.usIn2Pin, GPIO_PIN_SET);
          break;

        case MOTORENCODER_DIRECTION_BACKWARD:
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn1Port,
            xMotorEncoderPeripherals.usIn1Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn2Port,
            xMotorEncoderPeripherals.usIn2Pin, GPIO_PIN_RESET);
          break;

        case MOTORENCODER_DIRECTION_STOP:
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn1Port,
            xMotorEncoderPeripherals.usIn1Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn2Port,
            xMotorEncoderPeripherals.usIn2Pin, GPIO_PIN_RESET);
          break;

        case MOTORENCODER_DIRECTION_FREE:
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn1Port,
            xMotorEncoderPeripherals.usIn1Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn2Port,
            xMotorEncoderPeripherals.usIn2Pin, GPIO_PIN_SET);
          break;

        default:
          break;
      }
      break;

    case MOTORENCODER_MOTOR_LEFT:
      switch (xDirection) {
        case MOTORENCODER_DIRECTION_FORWARD:
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn3Port,
            xMotorEncoderPeripherals.usIn3Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn4Port,
            xMotorEncoderPeripherals.usIn4Pin, GPIO_PIN_RESET);
          break;

        case MOTORENCODER_DIRECTION_BACKWARD:
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn3Port,
            xMotorEncoderPeripherals.usIn3Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn4Port,
            xMotorEncoderPeripherals.usIn4Pin, GPIO_PIN_SET);
          break;

        case MOTORENCODER_DIRECTION_STOP:
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn3Port,
            xMotorEncoderPeripherals.usIn3Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn4Port,
            xMotorEncoderPeripherals.usIn4Pin, GPIO_PIN_RESET);
          break;

        case MOTORENCODER_DIRECTION_FREE:
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn3Port,
            xMotorEncoderPeripherals.usIn3Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(xMotorEncoderPeripherals.pIn4Port,
            xMotorEncoderPeripherals.usIn4Pin, GPIO_PIN_SET);
          break;

        default:
          break;
      }
      break;

    default:
      break;
  }
}

/**
 * @brief Initializes the motor control interface.
 *
 * This function must be called once during the application
 * initialization, before any motor command is sent. It stores the GPIO
 * pins and timer channels used to drive both motors and starts the PWM
 * outputs.
 *
 * @param pIn1Port GPIO port connected to IN1.
 * @param usIn1Pin GPIO pin connected to IN1.
 * @param pIn2Port GPIO port connected to IN2.
 * @param usIn2Pin GPIO pin connected to IN2.
 * @param pIn3Port GPIO port connected to IN3.
 * @param usIn3Pin GPIO pin connected to IN3.
 * @param pIn4Port GPIO port connected to IN4.
 * @param usIn4Pin GPIO pin connected to IN4.
 * @param pRightMotorTimerHandle Timer handle of the right motor PWM.
 * @param uiRightMotorTimerChannel Timer channel of the right motor PWM.
 * @param pLeftMotorTimerHandle Timer handle of the left motor PWM.
 * @param uiLeftMotorTimerChannel Timer channel of the left motor PWM.
 */
void vMotorEncoderInitMotors(GPIO_TypeDef *pIn1Port, uint16_t usIn1Pin,
  GPIO_TypeDef *pIn2Port, uint16_t usIn2Pin, GPIO_TypeDef *pIn3Port,
  uint16_t usIn3Pin, GPIO_TypeDef *pIn4Port, uint16_t usIn4Pin,
  TIM_HandleTypeDef *pRightMotorTimerHandle,
  unsigned int uiRightMotorTimerChannel,
  TIM_HandleTypeDef *pLeftMotorTimerHandle,
  unsigned int uiLeftMotorTimerChannel)
{
  xMotorEncoderPeripherals.pIn1Port = pIn1Port;
  xMotorEncoderPeripherals.usIn1Pin = usIn1Pin;
  xMotorEncoderPeripherals.pIn2Port = pIn2Port;
  xMotorEncoderPeripherals.usIn2Pin = usIn2Pin;
  xMotorEncoderPeripherals.pIn3Port = pIn3Port;
  xMotorEncoderPeripherals.usIn3Pin = usIn3Pin;
  xMotorEncoderPeripherals.pIn4Port = pIn4Port;
  xMotorEncoderPeripherals.usIn4Pin = usIn4Pin;
  xMotorEncoderPeripherals.pRightMotorTimerHandle =
    pRightMotorTimerHandle;
  xMotorEncoderPeripherals.uiRightMotorTimerChannel =
    uiRightMotorTimerChannel;
  xMotorEncoderPeripherals.pLeftMotorTimerHandle =
    pLeftMotorTimerHandle;
  xMotorEncoderPeripherals.uiLeftMotorTimerChannel =
    uiLeftMotorTimerChannel;

  vMotorEncoderControlMotor(MOTORENCODER_MOTOR_RIGHT,
    MOTORENCODER_DIRECTION_STOP, 0.0f);
  vMotorEncoderControlMotor(MOTORENCODER_MOTOR_LEFT,
    MOTORENCODER_DIRECTION_STOP, 0.0f);

  HAL_TIM_PWM_Start(xMotorEncoderPeripherals.pRightMotorTimerHandle,
    xMotorEncoderPeripherals.uiRightMotorTimerChannel);
  HAL_TIM_PWM_Start(xMotorEncoderPeripherals.pLeftMotorTimerHandle,
    xMotorEncoderPeripherals.uiLeftMotorTimerChannel);
}

/**
 * @brief Controls one motor direction and power.
 *
 * This function should be called whenever the application needs to
 * update the direction or the PWM duty cycle of one motor.
 *
 * @param xMotor Selected motor.
 * @param xDirection Direction applied to the selected motor.
 * @param fPower Power command applied to the selected motor.
 */
void vMotorEncoderControlMotor(motorEncoderMotor_t xMotor,
  motorEncoderDirection_t xDirection, float fPower)
{
  unsigned int uiCompareValue;

  vMotorEncoderSetDirection(xMotor, xDirection);
  uiCompareValue = (unsigned int)(fPower * MOTORENCODER_PWM_COMPARE_MAX);

  switch (xMotor) {
    case MOTORENCODER_MOTOR_RIGHT:
      __HAL_TIM_SET_COMPARE(
        xMotorEncoderPeripherals.pRightMotorTimerHandle,
        xMotorEncoderPeripherals.uiRightMotorTimerChannel,
        uiCompareValue);
      break;

    case MOTORENCODER_MOTOR_LEFT:
      __HAL_TIM_SET_COMPARE(
        xMotorEncoderPeripherals.pLeftMotorTimerHandle,
        xMotorEncoderPeripherals.uiLeftMotorTimerChannel,
        uiCompareValue);
      break;

    default:
      break;
  }
}

/**
 * @brief Initializes the encoder reading interface.
 *
 * This function must be called once during the application
 * initialization, after the timer peripherals are configured and before
 * reading encoder velocities. It stores the timer handles and starts
 * the interrupts required for encoder measurement.
 *
 * @param pRightEncoderTimerHandle Timer handle of the right encoder.
 * @param uiRightEncoderTimerChannel Timer channel of the right encoder.
 * @param pLeftEncoderTimerHandle Timer handle of the left encoder.
 * @param uiLeftEncoderTimerChannel Timer channel of the left encoder.
 */
void vMotorEncoderInitEncoders(
  TIM_HandleTypeDef *pRightEncoderTimerHandle,
  unsigned int uiRightEncoderTimerChannel,
  TIM_HandleTypeDef *pLeftEncoderTimerHandle,
  unsigned int uiLeftEncoderTimerChannel)
{
  xMotorEncoderPeripherals.pRightEncoderTimerHandle =
    pRightEncoderTimerHandle;
  xMotorEncoderPeripherals.uiRightEncoderTimerChannel =
    uiRightEncoderTimerChannel;
  xMotorEncoderPeripherals.pLeftEncoderTimerHandle =
    pLeftEncoderTimerHandle;
  xMotorEncoderPeripherals.uiLeftEncoderTimerChannel =
    uiLeftEncoderTimerChannel;

  HAL_TIM_Base_Start_IT(xMotorEncoderPeripherals.pRightEncoderTimerHandle);
  HAL_TIM_Base_Start_IT(xMotorEncoderPeripherals.pLeftEncoderTimerHandle);
  HAL_TIM_IC_Start_IT(xMotorEncoderPeripherals.pRightEncoderTimerHandle,
    xMotorEncoderPeripherals.uiRightEncoderTimerChannel);
  HAL_TIM_IC_Start_IT(xMotorEncoderPeripherals.pLeftEncoderTimerHandle,
    xMotorEncoderPeripherals.uiLeftEncoderTimerChannel);
}

/**
 * @brief Reads the velocity of one motor.
 *
 * This function should be called after the encoder interface has
 * already been initialized and updated by the timer events.
 *
 * @param xMotor Selected motor.
 * @return Measured velocity of the selected motor.
 */
float fMotorEncoderReadVelocity(motorEncoderMotor_t xMotor)
{
  switch (xMotor) {
    case MOTORENCODER_MOTOR_RIGHT:
      return xMotorEncoderAttributes.fRightMotorVelocity;

    case MOTORENCODER_MOTOR_LEFT:
      return xMotorEncoderAttributes.fLeftMotorVelocity;

    default:
      return 0.0f;
  }
}

/**
 * @brief Handles the encoder timer overflow event.
 *
 * This function must be called inside the timer period elapsed callback
 * associated with the encoder timer of the selected motor. It should be
 * called every time that timer overflows while waiting for the next
 * encoder pulse.
 *
 * @param xMotor Selected motor.
 */
void vMotorEncoderHandleTimerReset(motorEncoderMotor_t xMotor)
{
  switch (xMotor) {
    case MOTORENCODER_MOTOR_RIGHT:
      xMotorEncoderAttributes.uiRightResetCount++;

      if (xMotorEncoderAttributes.uiRightResetCount >
          MOTORENCODER_MAX_TIMER_RESET_COUNT) {
        xMotorEncoderAttributes.fRightMotorVelocity = 0.0f;
        xMotorEncoderAttributes.uiRightResetCount = 0U;
      }
      break;

    case MOTORENCODER_MOTOR_LEFT:
      xMotorEncoderAttributes.uiLeftResetCount++;

      if (xMotorEncoderAttributes.uiLeftResetCount >
          MOTORENCODER_MAX_TIMER_RESET_COUNT) {
        xMotorEncoderAttributes.fLeftMotorVelocity = 0.0f;
        xMotorEncoderAttributes.uiLeftResetCount = 0U;
      }
      break;

    default:
      break;
  }
}

/**
 * @brief Handles the encoder input capture event.
 *
 * This function must be called inside the input capture callback
 * associated with the encoder timer of the selected motor. It should be
 * called whenever a valid encoder capture event occurs.
 *
 * @param xMotor Selected motor.
 */
void vMotorEncoderHandleTimerCapture(motorEncoderMotor_t xMotor)
{
  long long llCount;

  llCount = 0;

  switch (xMotor) {
    case MOTORENCODER_MOTOR_RIGHT:
      HAL_TIM_Base_Stop_IT(xMotorEncoderPeripherals.pRightEncoderTimerHandle);
      HAL_TIM_IC_Stop_IT(xMotorEncoderPeripherals.pRightEncoderTimerHandle,
        xMotorEncoderPeripherals.uiRightEncoderTimerChannel);

      xMotorEncoderAttributes.uiRightCapturedValue =
        HAL_TIM_ReadCapturedValue(
          xMotorEncoderPeripherals.pRightEncoderTimerHandle,
          xMotorEncoderPeripherals.uiRightEncoderTimerChannel);

      llCount =
        (long long)xMotorEncoderAttributes.uiRightResetCount *
        MOTORENCODER_TIMER_MAX +
        xMotorEncoderAttributes.uiRightCapturedValue;

      xMotorEncoderAttributes.fRightMotorVelocity =
        MOTORENCODER_CM_PER_PULSE /
        ((float)llCount / MOTORENCODER_TIMER_CLOCK_HZ);

      xMotorEncoderAttributes.uiRightResetCount = 0U;

      __HAL_TIM_SET_COUNTER(
        xMotorEncoderPeripherals.pRightEncoderTimerHandle, 0U);

      HAL_TIM_Base_Start_IT(
        xMotorEncoderPeripherals.pRightEncoderTimerHandle);
      HAL_TIM_IC_Start_IT(xMotorEncoderPeripherals.pRightEncoderTimerHandle,
        xMotorEncoderPeripherals.uiRightEncoderTimerChannel);
      break;

    case MOTORENCODER_MOTOR_LEFT:
      HAL_TIM_Base_Stop_IT(xMotorEncoderPeripherals.pLeftEncoderTimerHandle);
      HAL_TIM_IC_Stop_IT(xMotorEncoderPeripherals.pLeftEncoderTimerHandle,
        xMotorEncoderPeripherals.uiLeftEncoderTimerChannel);

      xMotorEncoderAttributes.uiLeftCapturedValue =
        HAL_TIM_ReadCapturedValue(
          xMotorEncoderPeripherals.pLeftEncoderTimerHandle,
          xMotorEncoderPeripherals.uiLeftEncoderTimerChannel);

      llCount =
        (long long)xMotorEncoderAttributes.uiLeftResetCount *
        MOTORENCODER_TIMER_MAX +
        xMotorEncoderAttributes.uiLeftCapturedValue;

      xMotorEncoderAttributes.fLeftMotorVelocity =
        MOTORENCODER_CM_PER_PULSE /
        ((float)llCount / MOTORENCODER_TIMER_CLOCK_HZ);

      xMotorEncoderAttributes.uiLeftResetCount = 0U;

      __HAL_TIM_SET_COUNTER(
        xMotorEncoderPeripherals.pLeftEncoderTimerHandle, 0U);

      HAL_TIM_Base_Start_IT(
        xMotorEncoderPeripherals.pLeftEncoderTimerHandle);
      HAL_TIM_IC_Start_IT(xMotorEncoderPeripherals.pLeftEncoderTimerHandle,
        xMotorEncoderPeripherals.uiLeftEncoderTimerChannel);
      break;

    default:
      break;
  }
}
