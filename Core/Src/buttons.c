/**
 * @file buttons.c
 * @brief Button debouncing implementation.
 * @details Implements a GPIO and timer based debounce system for five
 * buttons.
 * @author Matheus
 * @date 2026-05-01
 * @version 1.1
 */

#include "buttons.h"

#define BUTTONS_FALSE 0U
#define BUTTONS_TRUE  1U

typedef struct {
  GPIO_TypeDef *pPort[BUTTONS_BUTTON_COUNT];
  uint16_t usPin[BUTTONS_BUTTON_COUNT];
  GPIO_PinState xLastState[BUTTONS_BUTTON_COUNT];
  uint16_t usDebouncingWindow;
  buttonsEnum_t xDebouncingButton;
  GPIO_PinState xDebouncingState;
  uint8_t ucDebouncingActive;
} buttonsAttributes_t;

static buttonsAttributes_t xButtonsAttributes;
static TIM_HandleTypeDef *pButtonsTimerHandler;

/**
 * @brief Initializes the button debounce module.
 * @details This function must be called once during the application
 * initialization, before enabling the button interrupts or using any
 * other function from this module. It stores the GPIO ports, GPIO pins,
 * timer handler, debounce window, and reads the initial state of each
 * button.
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
  TIM_HandleTypeDef *pTimerHandler, uint16_t usDebouncingWindow) {
  int i;
  int j;

  xButtonsAttributes.pPort[BUTTONS_BUTTON_UP] = pUpPort;
  xButtonsAttributes.pPort[BUTTONS_BUTTON_RIGHT] = pRightPort;
  xButtonsAttributes.pPort[BUTTONS_BUTTON_LEFT] = pLeftPort;
  xButtonsAttributes.pPort[BUTTONS_BUTTON_DOWN] = pDownPort;
  xButtonsAttributes.pPort[BUTTONS_BUTTON_ENTER] = pEnterPort;

  xButtonsAttributes.usPin[BUTTONS_BUTTON_UP] = usUpPin;
  xButtonsAttributes.usPin[BUTTONS_BUTTON_RIGHT] = usRightPin;
  xButtonsAttributes.usPin[BUTTONS_BUTTON_LEFT] = usLeftPin;
  xButtonsAttributes.usPin[BUTTONS_BUTTON_DOWN] = usDownPin;
  xButtonsAttributes.usPin[BUTTONS_BUTTON_ENTER] = usEnterPin;

  pButtonsTimerHandler = pTimerHandler;
  xButtonsAttributes.usDebouncingWindow = usDebouncingWindow;
  xButtonsAttributes.xDebouncingButton = BUTTONS_BUTTON_NONE;
  xButtonsAttributes.xDebouncingState = GPIO_PIN_RESET;
  xButtonsAttributes.ucDebouncingActive = BUTTONS_FALSE;

  for (i = 0; i < BUTTONS_BUTTON_COUNT; i++) {
    for (j = i + 1; j < BUTTONS_BUTTON_COUNT; j++) {
      if (xButtonsAttributes.usPin[i] == xButtonsAttributes.usPin[j]) {
        return BUTTONS_ERROR_REPEATED_PIN;
      }
    }
  }

  for (i = 0; i < BUTTONS_BUTTON_COUNT; i++) {
    xButtonsAttributes.xLastState[i] = HAL_GPIO_ReadPin(
      xButtonsAttributes.pPort[i], xButtonsAttributes.usPin[i]);
  }

  if (0U != xButtonsAttributes.usDebouncingWindow) {
    __HAL_TIM_SET_AUTORELOAD(pButtonsTimerHandler, xButtonsAttributes.usDebouncingWindow - 1U);
  }

  __HAL_TIM_SET_COUNTER(pButtonsTimerHandler, 0U);

  return BUTTONS_OK;
}

/**
 * @brief Gets the button associated with a GPIO pin.
 * @details Compares the received GPIO pin with the configured button
 * pins and returns the corresponding button identifier.
 * @param usButtonPin GPIO pin to search for.
 * @return Button associated with the pin, or BUTTONS_BUTTON_NONE.
 */
static buttonsEnum_t xButtonsGetButton(uint16_t usButtonPin) {
  buttonsEnum_t xButton;

  if (xButtonsAttributes.usPin[BUTTONS_BUTTON_UP] == usButtonPin) {
    xButton = BUTTONS_BUTTON_UP;
  } else if (xButtonsAttributes.usPin[BUTTONS_BUTTON_RIGHT] == usButtonPin) {
    xButton = BUTTONS_BUTTON_RIGHT;
  } else if (xButtonsAttributes.usPin[BUTTONS_BUTTON_LEFT] == usButtonPin) {
    xButton = BUTTONS_BUTTON_LEFT;
  } else if (xButtonsAttributes.usPin[BUTTONS_BUTTON_DOWN] == usButtonPin) {
    xButton = BUTTONS_BUTTON_DOWN;
  } else if (xButtonsAttributes.usPin[BUTTONS_BUTTON_ENTER] == usButtonPin) {
    xButton = BUTTONS_BUTTON_ENTER;
  } else {
    xButton = BUTTONS_BUTTON_NONE;
  }

  return xButton;
}

/**
 * @brief Starts the debouncing process for a GPIO interrupt pin.
 * @details This function must be called inside the GPIO external
 * interrupt callback, immediately after detecting an interrupt from one
 * of the configured button pins. It identifies the button associated
 * with the interrupt pin, stores the candidate GPIO state, and starts
 * the debounce timer.
 * @param usButtonPin GPIO pin that generated the external interrupt.
 */
void vButtonsDebouncingStart(uint16_t usButtonPin) {
  buttonsEnum_t xButton;
  GPIO_PinState xPinState;

  xButton = xButtonsGetButton(usButtonPin);

  if (BUTTONS_BUTTON_NONE == xButton) {
    return;
  }

  if (BUTTONS_TRUE == xButtonsAttributes.ucDebouncingActive) {
    return;
  }

  xPinState = HAL_GPIO_ReadPin(xButtonsAttributes.pPort[xButton],
    xButtonsAttributes.usPin[xButton]);

  xButtonsAttributes.xDebouncingButton = xButton;
  xButtonsAttributes.xDebouncingState = xPinState;
  xButtonsAttributes.ucDebouncingActive = BUTTONS_TRUE;

  if (0U != xButtonsAttributes.usDebouncingWindow) {
    __HAL_TIM_SET_AUTORELOAD(pButtonsTimerHandler, xButtonsAttributes.usDebouncingWindow - 1U);
  }

  __HAL_TIM_SET_COUNTER(pButtonsTimerHandler, 0U);
  HAL_TIM_Base_Start_IT(pButtonsTimerHandler);
}

/**
 * @brief Stops and validates the debouncing process.
 * @details This function must be called inside the timer period elapsed
 * callback associated with the debounce timer, after the configured
 * debounce window has elapsed. It stops the timer, validates the state
 * transition, and calls the corresponding application callback when a
 * stable button press or release is confirmed.
 */
void vButtonsDebouncingStop(void) {
  buttonsEnum_t xButton;
  GPIO_PinState xPinState;

  if (BUTTONS_FALSE == xButtonsAttributes.ucDebouncingActive) {
    return;
  }

  HAL_TIM_Base_Stop_IT(pButtonsTimerHandler);
  __HAL_TIM_SET_COUNTER(pButtonsTimerHandler, 0U);

  xButton = xButtonsAttributes.xDebouncingButton;

  if (BUTTONS_BUTTON_NONE == xButton) {
    xButtonsAttributes.ucDebouncingActive = BUTTONS_FALSE;
    return;
  }

  xPinState = HAL_GPIO_ReadPin(xButtonsAttributes.pPort[xButton], xButtonsAttributes.usPin[xButton]);

  xButtonsAttributes.ucDebouncingActive = BUTTONS_FALSE;

  if (xPinState != xButtonsAttributes.xDebouncingState) {
    return;
  }

  if (xPinState == xButtonsAttributes.xLastState[xButton]) {
    return;
  }

  xButtonsAttributes.xLastState[xButton] = xPinState;

  if (GPIO_PIN_SET == xPinState) {
    vButtonsPressedCallback(xButton);
  } else {
    vButtonsReleasedCallback(xButton);
  }

  xButtonsAttributes.xDebouncingButton = BUTTONS_BUTTON_NONE;
}

/**
 * @brief Callback called when a button is pressed.
 * @details This weak callback is called by vButtonsDebouncingStop after
 * the debounce timer expires and a valid stable press transition is
 * confirmed. It may be reimplemented by the application.
 * @param xButton Button that was pressed.
 */
__weak void vButtonsPressedCallback(buttonsEnum_t xButton) {
  (void)xButton;
}

/**
 * @brief Callback called when a button is released.
 * @details This weak callback is called by vButtonsDebouncingStop after
 * the debounce timer expires and a valid stable release transition is
 * confirmed. It may be reimplemented by the application.
 * @param xButton Button that was released.
 */
__weak void vButtonsReleasedCallback(buttonsEnum_t xButton) {
  (void)xButton;
}
