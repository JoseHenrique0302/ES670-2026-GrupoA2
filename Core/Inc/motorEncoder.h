/**
 * @file motorEncoder.h
 * @brief Declares the motor control and encoder reading interface.
 *
 * This file contains the constants, types, and function declarations
 * required to control the motors and read the encoder velocities.
 *
 * @author rodri
 * @date 19-Oct-2025
 * @version 1.0
 */

#ifndef MOTORENCODER_H
#define MOTORENCODER_H

#include "main.h"

#define MOTORENCODER_TIMER_MAX 65535U
#define MOTORENCODER_TIMER_CLOCK_HZ 1000000.0f
#define MOTORENCODER_TIMER_PERIOD_S \
  (MOTORENCODER_TIMER_MAX / MOTORENCODER_TIMER_CLOCK_HZ)

#define MOTORENCODER_PULSES_PER_REVOLUTION 20U
#define MOTORENCODER_WHEEL_CM_PER_REVOLUTION 22.0f
#define MOTORENCODER_CM_PER_PULSE \
  (MOTORENCODER_WHEEL_CM_PER_REVOLUTION / \
  MOTORENCODER_PULSES_PER_REVOLUTION)

#define MOTORENCODER_ZERO_TIMEOUT_S 0.5f
#define MOTORENCODER_MAX_TIMER_RESET_COUNT \
  (MOTORENCODER_ZERO_TIMEOUT_S / MOTORENCODER_TIMER_PERIOD_S)

typedef enum {
  MOTORENCODER_MOTOR_LEFT,
  MOTORENCODER_MOTOR_RIGHT
} motorEncoderMotor_t;

typedef enum {
  MOTORENCODER_DIRECTION_FORWARD,
  MOTORENCODER_DIRECTION_BACKWARD,
  MOTORENCODER_DIRECTION_STOP,
  MOTORENCODER_DIRECTION_FREE
} motorEncoderDirection_t;

typedef struct {
  unsigned int uiRightResetCount;
  unsigned int uiRightCapturedValue;
  float fRightMotorVelocity;
  unsigned int uiLeftResetCount;
  unsigned int uiLeftCapturedValue;
  float fLeftMotorVelocity;
} motorEncoderAttributes_t;

/**
 * @brief Initializes the motor control interface.
 *
 * This function must be called once during the application
 * initialization, before any motor command is sent. It stores the GPIO
 * pins and timer channels used to drive both motors.
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
  unsigned int uiLeftMotorTimerChannel);

/**
 * @brief Controls one motor direction and power.
 *
 * This function should be called whenever the application needs to
 * update the command applied to one motor.
 *
 * @param xMotor Selected motor.
 * @param xDirection Direction applied to the selected motor.
 * @param fPower Power command applied to the selected motor.
 */
void vMotorEncoderControlMotor(motorEncoderMotor_t xMotor,
  motorEncoderDirection_t xDirection, float fPower);

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
  unsigned int uiLeftEncoderTimerChannel);

/**
 * @brief Reads the velocity of one motor.
 *
 * This function should be called after the encoder interface has
 * already been initialized and updated by the timer events.
 *
 * @param xMotor Selected motor.
 * @return Measured velocity of the selected motor.
 */
float fMotorEncoderReadVelocity(motorEncoderMotor_t xMotor);

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
void vMotorEncoderHandleTimerReset(motorEncoderMotor_t xMotor);

/**
 * @brief Handles the encoder input capture event.
 *
 * This function must be called inside the input capture callback
 * associated with the encoder timer of the selected motor. It should be
 * called whenever a valid encoder capture event occurs.
 *
 * @param xMotor Selected motor.
 */
void vMotorEncoderHandleTimerCapture(motorEncoderMotor_t xMotor);

/**
 * @brief Reads the accumulated distance of one motor.
 * @param xMotor Selected motor.
 * @return Distance in centimeters.
 */
float fMotorEncoderReadDistance(motorEncoderMotor_t xMotor);

/**
 * @brief Resets the accumulated distance of one motor.
 * @param xMotor Selected motor.
 */
void vMotorEncoderResetDistance(motorEncoderMotor_t xMotor);

#endif // MOTORENCODER_H
