/**
 * @file buttons.h
 * @brief Declares the button debouncing interface.
 * @details This file contains the types, constants, and function
 * declarations required to initialize the button debouncing module and
 * process stable button press and release events.
 * @author Matheus
 * @date 2026-05-01
 * @version 1.1
 */

#ifndef BUTTONS_H
#define BUTTONS_H

#include "main.h"

#define BUTTONS_BUTTON_COUNT       5U

#define BUTTONS_OK                 0U
#define BUTTONS_ERROR_REPEATED_PIN 1U

typedef enum {
  BUTTONS_BUTTON_UP = 0,
  BUTTONS_BUTTON_RIGHT,
  BUTTONS_BUTTON_LEFT,
  BUTTONS_BUTTON_DOWN,
  BUTTONS_BUTTON_ENTER,
  BUTTONS_BUTTON_NONE
} buttonsEnum_t;

/**
 * @brief Initializes the button debounce module.
 * @details This function must be called once during the application
 * initialization, before enabling the button interrupts or calling any
 * other function from this module.
 * @param pUpPort GPIO port of the up button.
 * @param usUpPin GPIO pin of the up button.
 * @param pRightPort GPIO port of the right button.
 * @param usRightPin GPIO pin of the right button.
 * @param pLeftPort GPIO port of the left button.
 * @param usLeftPin GPIO pin of the left button.
 * @param pDownPort GPIO port of the down button.
 * @param usDownPin GPIO pin of the down button.
 * @param pEnterPort GPIO port of the enter button.
 * @param usEnterPin GPIO pin of the enter button.
 * @param pTimerHandler Timer handler used for debouncing.
 * @param usDebouncingWindow Debouncing window configured in timer ticks.
 * @return BUTTONS_OK if successful, or an error flag otherwise.
 */
uint8_t ucButtonsInit(GPIO_TypeDef *pUpPort, uint16_t usUpPin,
  GPIO_TypeDef *pRightPort, uint16_t usRightPin,
  GPIO_TypeDef *pLeftPort, uint16_t usLeftPin,
  GPIO_TypeDef *pDownPort, uint16_t usDownPin,
  GPIO_TypeDef *pEnterPort, uint16_t usEnterPin,
  TIM_HandleTypeDef *pTimerHandler, uint16_t usDebouncingWindow);

/**
 * @brief Starts the debouncing process for a GPIO interrupt pin.
 * @details This function must be called inside the GPIO external
 * interrupt callback, immediately after detecting an interrupt from one
 * of the configured button pins.
 * @param usButtonPin GPIO pin that generated the external interrupt.
 */
void vButtonsDebouncingStart(uint16_t usButtonPin);

/**
 * @brief Stops and validates the debouncing process.
 * @details This function must be called inside the timer period elapsed
 * callback associated with the debounce timer, after the configured
 * debounce window has elapsed.
 */
void vButtonsDebouncingStop(void);

/**
 * @brief Callback called when a button is pressed.
 * @details This weak callback is called by vButtonsDebouncingStop after
 * the debounce timer expires and a valid stable press transition is
 * confirmed. It may be reimplemented by the application.
 * @param xButton Button that was pressed.
 */
void vButtonsPressedCallback(buttonsEnum_t xButton);

/**
 * @brief Callback called when a button is released.
 * @details This weak callback is called by vButtonsDebouncingStop after
 * the debounce timer expires and a valid stable release transition is
 * confirmed. It may be reimplemented by the application.
 * @param xButton Button that was released.
 */
void vButtonsReleasedCallback(buttonsEnum_t xButton);

#endif // BUTTONS_H
