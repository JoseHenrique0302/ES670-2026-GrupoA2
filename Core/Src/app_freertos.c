/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "battery.h"
#include "lineSensors.h"
#include "motorEncoder.h"
#include "pid.h"
#include "distanceSensor.h"
#include "lcd_hd44780_i2c.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Estrutura para os dados dos sensores (Q1)
typedef struct {
    uint16_t usIRRaw[5];      // leitura crua dos 5 sensores IR (0-4095)
    uint8_t  ucIRBin[5];      // valor binarizado (0=branco, 1=preto)
    uint16_t usBatteryRaw;    // tensão da bateria em mV (ou ADC raw)
    uint8_t  ucBatteryPct;    // percentual de carga da bateria
} SensorsData_t;

// Estrutura para o comando dos motores (Q2)
typedef struct {
    float fSpeedLeft;         // velocidade esquerda (m/s ou RPM)
    float fSpeedRight;        // velocidade direita (m/s ou RPM)
    uint8_t ucCmdType;        // 0=STOP, 1=AUTO, 2=MANUAL, etc.
} MotorCommand_t;

// Estrutura para o comando de modo (Q4)
typedef enum {
    MODE_MANUAL = 0,
    MODE_AUTONOMOUS = 1
} SystemMode_t;

// Estrutura para os eventos de botão (Q5)
typedef struct {
    uint8_t ucButtonId;       // 0=UP,1=RIGHT,2=LEFT,3=DOWN,4=ENTER
    uint8_t ucEventType;      // 0=PRESS, 1=RELEASE
} ButtonEvent_t;

// Estrutura para dados de telemetria (GV5)
typedef struct {
    float    posX;             // coordenada x (m)
    float    posY;             // coordenada y (m)
    float    theta;            // orientação (rad)
    float    speedCurrent;     // velocidade atual (m/s)
    float    speedAverage;     // velocidade média (m/s)
    float    distTotal;        // distância percorrida (m)
    uint8_t  batteryPct;       // bateria estimada (%)
    uint8_t  systemMode;       // MANUAL=0, AUTONOMOUS=1
    uint8_t  calibDone;        // flag: calibração concluída
    uint8_t  reserved;         // alinhamento
} TelemetryData_t;

// Estrutura para dados de calibração persistidos (GV3)
typedef struct {
    uint16_t usIRThreshold[5]; // limiares para binarização dos IR (0-4095)
    float    fPidKp;           // ganho proporcional padrão
    float    fPidKi;           // ganho integral padrão
    float    fPidKd;           // ganho derivativo padrão
    uint16_t usEncoderPPR;     // pulsos por revolução do encoder
} CalibData_t;

// Estrutura para parâmetros PID atuais (GV2)
typedef struct {
    float fKp;
    float fKi;
    float fKd;
} PidParams_t;

// Definição dos bits do event group evEmergency
#define EMERGENCY_BIT   (1 << 0)

// Definição dos bits do event group evButtonsState (5 botões)
#define BUTTON_UP_BIT    (1 << 0)
#define BUTTON_RIGHT_BIT (1 << 1)
#define BUTTON_LEFT_BIT  (1 << 2)
#define BUTTON_DOWN_BIT  (1 << 3)
#define BUTTON_ENTER_BIT (1 << 4)

// Definição das prioridades e períodos (em ms)
#define TASK_PERIOD_SENSORS    50
#define TASK_PERIOD_ODOMETRY   50
#define TASK_PERIOD_LINE_PID   100
#define TASK_PERIOD_BUTTONS    50
#define TASK_PERIOD_UART       50
#define TASK_PERIOD_LCD       1000
#define TASK_PERIOD_ULTRASONIC 50

// Definição dos tamanhos das filas
#define QUEUE_SIZE_SENSORS      5
#define QUEUE_SIZE_MOTOR_CMD    8
#define QUEUE_SIZE_MODE_CMD     2
#define QUEUE_SIZE_BUTTONS      4
#define QUEUE_SIZE_SYSTEM_STATUS 4
#define QUEUE_SIZE_LCD_DATA     12

// Definição do timeout para as filas
#define QUEUE_TIMEOUT_MS        100

// Outros defines
#define MAX_ENCODER_COUNTS      1000000UL
#define MIN_BATTERY_VOLTAGE_MV  5500   // 5.5V
#define STOP_DISTANCE_CM        5.0f

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// Variáveis globais conforme diagrama
static volatile int32_t gvEncoderCounts[2] = {0, 0};   // GV1: [0]=esquerda, [1]=direita
static CalibData_t gvCalibData;                        // GV3: dados de calibração
static PidParams_t gvPIDParams;                        // GV2: ganhos PID atuais
static TelemetryData_t gvTelemetry;                    // GV5: telemetria
static volatile uint32_t g_ultrasonicEchoTicks = 0;    // GV4: tempo do eco em ticks

// Handle do LCD (definido em lcd_hd44780_i2c.c, mas declarado aqui como extern)
extern I2C_HandleTypeDef hi2c2;

// Handles dos timers e UART (gerados pelo CubeMX)
extern TIM_HandleTypeDef htim1, htim8, htim16, htim17, htim20, htim3;
extern UART_HandleTypeDef hlpuart1;

// Definição dos pinos dos motores (conforme gpio.c)
#define MOTOR_DIR_IN1_Pin   Motor_Dir_IN1_Pin
#define MOTOR_DIR_IN1_GPIO_Port Motor_Dir_IN1_GPIO_Port
#define MOTOR_DIR_IN2_Pin   Motor_Dir_IN2_Pin
#define MOTOR_DIR_IN2_GPIO_Port Motor_Dir_IN2_GPIO_Port
#define MOTOR_ESQ_IN3_Pin   Motor_Esq_IN3_Pin
#define MOTOR_ESQ_IN3_GPIO_Port Motor_Esq_IN3_GPIO_Port
#define MOTOR_ESQ_IN4_Pin   Motor_Esq_IN4_Pin
#define MOTOR_ESQ_IN4_GPIO_Port Motor_Esq_IN4_GPIO_Port

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for vTaskCalibracao */
osThreadId_t vTaskCalibracaoHandle;
const osThreadAttr_t vTaskCalibracao_attributes = {
  .name = "vTaskCalibracao",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for vTaskMotor */
osThreadId_t vTaskMotorHandle;
const osThreadAttr_t vTaskMotor_attributes = {
  .name = "vTaskMotor",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};
/* Definitions for vTaskSensorRead */
osThreadId_t vTaskSensorReadHandle;
const osThreadAttr_t vTaskSensorRead_attributes = {
  .name = "vTaskSensorRead",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for vTaskSegueLinha */
osThreadId_t vTaskSegueLinhaHandle;
const osThreadAttr_t vTaskSegueLinha_attributes = {
  .name = "vTaskSegueLinha",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for vTaskOdometria */
osThreadId_t vTaskOdometriaHandle;
const osThreadAttr_t vTaskOdometria_attributes = {
  .name = "vTaskOdometria",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for vTaskUART */
osThreadId_t vTaskUARTHandle;
const osThreadAttr_t vTaskUART_attributes = {
  .name = "vTaskUART",
  .priority = (osPriority_t) osPriorityBelowNormal4,
  .stack_size = 128 * 4
};
/* Definitions for vTaskLCD */
osThreadId_t vTaskLCDHandle;
const osThreadAttr_t vTaskLCD_attributes = {
  .name = "vTaskLCD",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for vTaskButtons */
osThreadId_t vTaskButtonsHandle;
const osThreadAttr_t vTaskButtons_attributes = {
  .name = "vTaskButtons",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for vTaskTrocarModo */
osThreadId_t vTaskTrocarModoHandle;
const osThreadAttr_t vTaskTrocarModo_attributes = {
  .name = "vTaskTrocarModo",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for vTaskUltraBuzz */
osThreadId_t vTaskUltraBuzzHandle;
const osThreadAttr_t vTaskUltraBuzz_attributes = {
  .name = "vTaskUltraBuzz",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for qSensorsData */
osMessageQueueId_t qSensorsDataHandle;
const osMessageQueueAttr_t qSensorsData_attributes = {
  .name = "qSensorsData"
};
/* Definitions for qMotorCommand */
osMessageQueueId_t qMotorCommandHandle;
const osMessageQueueAttr_t qMotorCommand_attributes = {
  .name = "qMotorCommand"
};
/* Definitions for qTrocaModo */
osMessageQueueId_t qTrocaModoHandle;
const osMessageQueueAttr_t qTrocaModo_attributes = {
  .name = "qTrocaModo"
};
/* Definitions for qButtonsEvent */
osMessageQueueId_t qButtonsEventHandle;
const osMessageQueueAttr_t qButtonsEvent_attributes = {
  .name = "qButtonsEvent"
};
/* Definitions for qSystemStatus */
osMessageQueueId_t qSystemStatusHandle;
const osMessageQueueAttr_t qSystemStatus_attributes = {
  .name = "qSystemStatus"
};
/* Definitions for qLCDData */
osMessageQueueId_t qLCDDataHandle;
const osMessageQueueAttr_t qLCDData_attributes = {
  .name = "qLCDData"
};
/* Definitions for mutexTelemetry */
osMutexId_t mutexTelemetryHandle;
const osMutexAttr_t mutexTelemetry_attributes = {
  .name = "mutexTelemetry"
};
/* Definitions for mutexCalibData */
osMutexId_t mutexCalibDataHandle;
const osMutexAttr_t mutexCalibData_attributes = {
  .name = "mutexCalibData"
};
/* Definitions for mutexPIDParams */
osMutexId_t mutexPIDParamsHandle;
const osMutexAttr_t mutexPIDParams_attributes = {
  .name = "mutexPIDParams"
};
/* Definitions for semUltrassonic */
osSemaphoreId_t semUltrassonicHandle;
const osSemaphoreAttr_t semUltrassonic_attributes = {
  .name = "semUltrassonic"
};
/* Definitions for semBTRxReady */
osSemaphoreId_t semBTRxReadyHandle;
const osSemaphoreAttr_t semBTRxReady_attributes = {
  .name = "semBTRxReady"
};
/* Definitions for evEmergency */
osEventFlagsId_t evEmergencyHandle;
const osEventFlagsAttr_t evEmergency_attributes = {
  .name = "evEmergency"
};
/* Definitions for evButtonsState */
osEventFlagsId_t evButtonsStateHandle;
const osEventFlagsAttr_t evButtonsState_attributes = {
  .name = "evButtonsState"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void vStartTaskCalibration(void *argument);
void vStartTaskMotor(void *argument);
void vStartTaskSensorRead(void *argument);
void vStartTaskSegueLinha(void *argument);
void vStartTaskOdometria(void *argument);
void vStartTaskUART(void *argument);
void vStartTaskLCD(void *argument);
void vStartTaskButtons(void *argument);
void vStartTaskTrocarModo(void *argument);
void vStartTaskUltrassonicBuzzer(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    // Inicializa variáveis globais
    memset((void*)&gvCalibData, 0, sizeof(CalibData_t));
    memset((void*)&gvPIDParams, 0, sizeof(PidParams_t));
    memset((void*)&gvTelemetry, 0, sizeof(TelemetryData_t));
    gvEncoderCounts[0] = 0;
    gvEncoderCounts[1] = 0;
    g_ultrasonicEchoTicks = 0;

    // Inicializa os periféricos
    vLineSensorsInit();
    vBatteryInit();
    vMotorEncoderInitMotors(MOTOR_DIR_IN1_GPIO_Port, MOTOR_DIR_IN1_Pin,
                            MOTOR_DIR_IN2_GPIO_Port, MOTOR_DIR_IN2_Pin,
                            MOTOR_ESQ_IN3_GPIO_Port, MOTOR_ESQ_IN3_Pin,
                            MOTOR_ESQ_IN4_GPIO_Port, MOTOR_ESQ_IN4_Pin,
                            &htim8, TIM_CHANNEL_1,   // motor direito
                            &htim1, TIM_CHANNEL_1);  // motor esquerdo
    vMotorEncoderInitEncoders(&htim16, TIM_CHANNEL_1, &htim17, TIM_CHANNEL_1);

    // Inicializa o LCD
    lcdInit(&hi2c2, 0x27, 2, 16);
    lcdBacklight(LCD_BIT_BACKIGHT_ON);
    lcdCommand(LCD_DISPLAY, LCD_PARAM_SET);
    lcdCommand(LCD_CURSOR, LCD_PARAM_UNSET);
    lcdCommand(LCD_CURSOR_BLINK, LCD_PARAM_UNSET);
    lcdPrintStr((uint8_t*)"Robot Ready", 11);

    // Inicializa o sensor ultrassônico (assumindo TIM20 como trigger, TIM3 como echo)
    vDistanceSensorInit(&htim20, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_1);
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of mutexTelemetry */
  mutexTelemetryHandle = osMutexNew(&mutexTelemetry_attributes);

  /* creation of mutexCalibData */
  mutexCalibDataHandle = osMutexNew(&mutexCalibData_attributes);

  /* creation of mutexPIDParams */
  mutexPIDParamsHandle = osMutexNew(&mutexPIDParams_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semUltrassonic */
  semUltrassonicHandle = osSemaphoreNew(1, 0, &semUltrassonic_attributes);

  /* creation of semBTRxReady */
  semBTRxReadyHandle = osSemaphoreNew(128, 0, &semBTRxReady_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of qSensorsData */
  qSensorsDataHandle = osMessageQueueNew (5, sizeof(uint16_t), &qSensorsData_attributes);

  /* creation of qMotorCommand */
  qMotorCommandHandle = osMessageQueueNew (8, sizeof(uint16_t), &qMotorCommand_attributes);

  /* creation of qTrocaModo */
  qTrocaModoHandle = osMessageQueueNew (2, sizeof(uint8_t), &qTrocaModo_attributes);

  /* creation of qButtonsEvent */
  qButtonsEventHandle = osMessageQueueNew (4, sizeof(uint8_t), &qButtonsEvent_attributes);

  /* creation of qSystemStatus */
  qSystemStatusHandle = osMessageQueueNew (4, sizeof(uint8_t), &qSystemStatus_attributes);

  /* creation of qLCDData */
  qLCDDataHandle = osMessageQueueNew (12, sizeof(uint64_t), &qLCDData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of vTaskCalibracao */
  vTaskCalibracaoHandle = osThreadNew(vStartTaskCalibration, NULL, &vTaskCalibracao_attributes);

  /* creation of vTaskMotor */
  vTaskMotorHandle = osThreadNew(vStartTaskMotor, NULL, &vTaskMotor_attributes);

  /* creation of vTaskSensorRead */
  vTaskSensorReadHandle = osThreadNew(vStartTaskSensorRead, NULL, &vTaskSensorRead_attributes);

  /* creation of vTaskSegueLinha */
  vTaskSegueLinhaHandle = osThreadNew(vStartTaskSegueLinha, NULL, &vTaskSegueLinha_attributes);

  /* creation of vTaskOdometria */
  vTaskOdometriaHandle = osThreadNew(vStartTaskOdometria, NULL, &vTaskOdometria_attributes);

  /* creation of vTaskUART */
  vTaskUARTHandle = osThreadNew(vStartTaskUART, NULL, &vTaskUART_attributes);

  /* creation of vTaskLCD */
  vTaskLCDHandle = osThreadNew(vStartTaskLCD, NULL, &vTaskLCD_attributes);

  /* creation of vTaskButtons */
  vTaskButtonsHandle = osThreadNew(vStartTaskButtons, NULL, &vTaskButtons_attributes);

  /* creation of vTaskTrocarModo */
  vTaskTrocarModoHandle = osThreadNew(vStartTaskTrocarModo, NULL, &vTaskTrocarModo_attributes);

  /* creation of vTaskUltraBuzz */
  vTaskUltraBuzzHandle = osThreadNew(vStartTaskUltrassonicBuzzer, NULL, &vTaskUltraBuzz_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of evEmergency */
  evEmergencyHandle = osEventFlagsNew(&evEmergency_attributes);

  /* creation of evButtonsState */
  evButtonsStateHandle = osEventFlagsNew(&evButtonsState_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_vStartTaskCalibration */
/**
* @brief Function implementing the vTaskCalibracao thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartTaskCalibration */
void vStartTaskCalibration(void *argument)
{
  /* USER CODE BEGIN vStartTaskCalibration */

	    (void)argument;
	    uint8_t ucStatus = 0x01; // CALIBRATION_DONE

	    for(;;)
	    {
	        vTaskSuspend(NULL);   // aguarda ser resumida por T7

	        // Reseta e atualiza calibração
	        vLineSensorsResetCalibration();
	        for (int i = 0; i < 20; i++)   // 200ms de amostragem
	        {
	            vLineSensorsUpdateCalibration();
	            vTaskDelay(pdMS_TO_TICKS(10));
	        }

	        // Salva os limiares em GV3
	        if (osMutexAcquire(mutexCalibDataHandle, pdMS_TO_TICKS(100)) == osOK)
	        {
	            for (int i = 0; i < 5; i++)
	            {
	                uint16_t usMax = fLineSensorGetMax((lineSensorsEnum_t)i);
	                uint16_t usMin = fLineSensorGetMin((lineSensorsEnum_t)i);
	                gvCalibData.usIRThreshold[i] = (usMax + usMin) / 2;
	            }
	            osMutexRelease(mutexCalibDataHandle);
	        }

	        // Atualiza flag na telemetria
	        if (osMutexAcquire(mutexTelemetryHandle, pdMS_TO_TICKS(100)) == osOK)
	        {
	            gvTelemetry.calibDone = 1;
	            osMutexRelease(mutexTelemetryHandle);
	        }

	        osMessageQueuePut(qSystemStatusHandle, &ucStatus, 0, 0);
	        // Volta a suspender
	        vTaskSuspend(NULL);
	    }


  /* USER CODE END vStartTaskCalibration */
}

/* USER CODE BEGIN Header_vStartTaskMotor */
/**
* @brief Function implementing the vTaskMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartTaskMotor */
void vStartTaskMotor(void *argument)
{
  /* USER CODE BEGIN vStartTaskMotor */
	    (void)argument;
	    MotorCommand_t xCmd;
	    const TickType_t xPeriod = pdMS_TO_TICKS(10);
	    TickType_t xLastWakeTime = xTaskGetTickCount();

	    for(;;)
	    {
	        vTaskDelayUntil(&xLastWakeTime, xPeriod);

	        // Verifica emergência
	        uint32_t ulEmergencyFlags = osEventFlagsGet(evEmergencyHandle);
	        if (ulEmergencyFlags & EMERGENCY_BIT)
	        {
	            vMotorEncoderControlMotor(MOTORENCODER_MOTOR_LEFT, MOTORENCODER_DIRECTION_STOP, 0.0f);
	            vMotorEncoderControlMotor(MOTORENCODER_MOTOR_RIGHT, MOTORENCODER_DIRECTION_STOP, 0.0f);
	            // Aguarda até que o bit seja limpo (opcional)
	            osEventFlagsWait(evEmergencyHandle, EMERGENCY_BIT, osFlagsWaitAny | osFlagsNoClear, portMAX_DELAY);
	            continue;
	        }

	        // Tenta receber comando da Q2 (timeout de 50ms)
	        if (osMessageQueueGet(qMotorCommandHandle, &xCmd, NULL, pdMS_TO_TICKS(50)) == osOK)
	        {
	            if (xCmd.ucCmdType == 0) // STOP
	            {
	                vMotorEncoderControlMotor(MOTORENCODER_MOTOR_LEFT, MOTORENCODER_DIRECTION_STOP, 0.0f);
	                vMotorEncoderControlMotor(MOTORENCODER_MOTOR_RIGHT, MOTORENCODER_DIRECTION_STOP, 0.0f);
	            }
	            else
	            {
	                vMotorEncoderControlMotor(MOTORENCODER_MOTOR_LEFT,
	                    (xCmd.fSpeedLeft >= 0) ? MOTORENCODER_DIRECTION_FORWARD : MOTORENCODER_DIRECTION_BACKWARD,
	                    (xCmd.fSpeedLeft > 0) ? xCmd.fSpeedLeft : -xCmd.fSpeedLeft);
	                vMotorEncoderControlMotor(MOTORENCODER_MOTOR_RIGHT,
	                    (xCmd.fSpeedRight >= 0) ? MOTORENCODER_DIRECTION_FORWARD : MOTORENCODER_DIRECTION_BACKWARD,
	                    (xCmd.fSpeedRight > 0) ? xCmd.fSpeedRight : -xCmd.fSpeedRight);
	            }
	        }
	    }

  /* USER CODE END vStartTaskMotor */
}

/* USER CODE BEGIN Header_vStartTaskSensorRead */
/**
* @brief Function implementing the vTaskSensorRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartTaskSensorRead */
void vStartTaskSensorRead(void *argument)
{
  /* USER CODE BEGIN vStartTaskSensorRead */

	    (void)argument;
	    SensorsData_t xData;
	    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_SENSORS);
	    TickType_t xLastWakeTime = xTaskGetTickCount();

	    for(;;)
	    {
	        vTaskDelayUntil(&xLastWakeTime, xPeriod);

	        // 1. Lê valores crus dos sensores de linha
	        for (int i = 0; i < 5; i++)
	            xData.usIRRaw[i] = fLineSensorGetRawValue((lineSensorsEnum_t)i);

	        // 2. Binariza usando limiares de GV3
	        if (osMutexAcquire(mutexCalibDataHandle, pdMS_TO_TICKS(5)) == osOK)
	        {
	            for (int i = 0; i < 5; i++)
	                xData.ucIRBin[i] = (xData.usIRRaw[i] > gvCalibData.usIRThreshold[i]) ? 1 : 0;
	            osMutexRelease(mutexCalibDataHandle);
	        }

	        // 3. Lê bateria
	        xData.usBatteryRaw = usBatteryGetRawValue();
	        xData.ucBatteryPct = usBatteryGetCharge();

	        // 4. Publica na Q1 (overwrite)
	        osMessageQueuePut(qSensorsDataHandle, &xData, 0, 0);

	        // 5. Atualiza telemetria
	        if (osMutexAcquire(mutexTelemetryHandle, pdMS_TO_TICKS(5)) == osOK)
	        {
	            gvTelemetry.batteryPct = xData.ucBatteryPct;
	            osMutexRelease(mutexTelemetryHandle);
	        }

	        // 6. Verifica bateria baixa e seta emergência
	        uint16_t usBattery_mV = (xData.usBatteryRaw * 3300) / 4095;
	        if (usBattery_mV < MIN_BATTERY_VOLTAGE_MV)
	            osEventFlagsSet(evEmergencyHandle, EMERGENCY_BIT);
	    }

  /* USER CODE END vStartTaskSensorRead */
}

/* USER CODE BEGIN Header_vStartTaskSegueLinha */
/**
* @brief Function implementing the vTaskSegueLinha thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartTaskSegueLinha */
void vStartTaskSegueLinha(void *argument)
{
  /* USER CODE BEGIN vStartTaskSegueLinha */

	    (void)argument;
	    SensorsData_t xSensors;
	    MotorCommand_t xCmd;
	    float fError, fOutput;
	    static pid_data_type xPidInstance;
	    static uint8_t ucFirstRun = 1;
	    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_LINE_PID);
	    TickType_t xLastWakeTime = xTaskGetTickCount();

	    // Inicialização do PID
	    if (ucFirstRun)
	    {
	        PidParams_t localPid;
	        if (osMutexAcquire(mutexPIDParamsHandle, pdMS_TO_TICKS(100)) == osOK)
	        {
	            localPid = gvPIDParams;
	            osMutexRelease(mutexPIDParamsHandle);
	        }
	        else
	            localPid = (PidParams_t){0.5f, 0.0f, 0.0f};
	        vPidInit(&xPidInstance, localPid.fKp, localPid.fKi, localPid.fKd, 10.0f, 100.0f);
	        ucFirstRun = 0;
	    }

	    for(;;)
	    {
	        vTaskDelayUntil(&xLastWakeTime, xPeriod);

	        // Recebe dados da Q1 (timeout 0)
	        if (osMessageQueueGet(qSensorsDataHandle, &xSensors, NULL, 0) != osOK)
	            continue;

	        // Calcula erro: posição da linha com pesos -2 a +2
	        fError = 0.0f;
	        int8_t weights[5] = {-2, -1, 0, 1, 2};
	        uint8_t activeCount = 0;
	        for (int i = 0; i < 5; i++)
	        {
	            if (xSensors.ucIRBin[i])
	            {
	                fError += weights[i];
	                activeCount++;
	            }
	        }
	        if (activeCount) fError /= activeCount;

	        // Atualiza ganhos PID se modificados
	        if (osMutexAcquire(mutexPIDParamsHandle, pdMS_TO_TICKS(5)) == osOK)
	        {
	            if (gvPIDParams.fKp != xPidInstance.fKp ||
	                gvPIDParams.fKi != xPidInstance.fKi ||
	                gvPIDParams.fKd != xPidInstance.fKd)
	            {
	                vPidSetKp(&xPidInstance, gvPIDParams.fKp);
	                vPidSetKi(&xPidInstance, gvPIDParams.fKi);
	                vPidSetKd(&xPidInstance, gvPIDParams.fKd);
	            }
	            osMutexRelease(mutexPIDParamsHandle);
	        }

	        fOutput = fPidUpdateData(&xPidInstance, 0.0f, fError);
	        float fBaseSpeed = 0.5f;
	        float fTurn = fOutput / 100.0f;
	        if (fTurn > 0.3f) fTurn = 0.3f;
	        if (fTurn < -0.3f) fTurn = -0.3f;

	        xCmd.fSpeedLeft = fBaseSpeed + fTurn;
	        xCmd.fSpeedRight = fBaseSpeed - fTurn;
	        xCmd.ucCmdType = 1; // AUTO
	        osMessageQueuePut(qMotorCommandHandle, &xCmd, 0, 0);
	    }

  /* USER CODE END vStartTaskSegueLinha */
}

/* USER CODE BEGIN Header_vStartTaskOdometria */
/**
* @brief Function implementing the vTaskOdometria thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartTaskOdometria */
void vStartTaskOdometria(void *argument)
{
  /* USER CODE BEGIN vStartTaskOdometria */

	    (void)argument;
	    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_ODOMETRY);
	    TickType_t xLastWakeTime = xTaskGetTickCount();
	    static float fLastLeftCount = 0, fLastRightCount = 0;
	    const float WHEEL_BASE = 0.2f;
	    const float WHEEL_RADIUS = 0.03f;
	    const float PPR = 400.0f;
	    static float fSpeedSum = 0.0f;
	    static int speedCount = 0;
	    char lcdLine1[17], lcdLine2[17];
	    uint8_t lcdBuffer[32];

	    for(;;)
	    {
	        vTaskDelayUntil(&xLastWakeTime, xPeriod);

	        int32_t leftCount, rightCount;
	        taskENTER_CRITICAL();
	        leftCount = gvEncoderCounts[0];
	        rightCount = gvEncoderCounts[1];
	        taskEXIT_CRITICAL();

	        float fDeltaLeft = (leftCount - fLastLeftCount) * (2.0f * 3.14159f * WHEEL_RADIUS) / PPR;
	        float fDeltaRight = (rightCount - fLastRightCount) * (2.0f * 3.14159f * WHEEL_RADIUS) / PPR;
	        fLastLeftCount = leftCount;
	        fLastRightCount = rightCount;

	        float fDeltaDist = (fDeltaLeft + fDeltaRight) / 2.0f;
	        float fDeltaTheta = (fDeltaRight - fDeltaLeft) / WHEEL_BASE;

	        if (osMutexAcquire(mutexTelemetryHandle, pdMS_TO_TICKS(5)) == osOK)
	        {
	            gvTelemetry.posX += fDeltaDist * cosf(gvTelemetry.theta);
	            gvTelemetry.posY += fDeltaDist * sinf(gvTelemetry.theta);
	            gvTelemetry.theta += fDeltaTheta;
	            // Normaliza theta
	            if (gvTelemetry.theta > 3.14159f) gvTelemetry.theta -= 2.0f * 3.14159f;
	            if (gvTelemetry.theta < -3.14159f) gvTelemetry.theta += 2.0f * 3.14159f;

	            float fSpeed = fDeltaDist / (TASK_PERIOD_ODOMETRY / 1000.0f);
	            gvTelemetry.speedCurrent = fSpeed;
	            gvTelemetry.distTotal += fabsf(fDeltaDist);

	            fSpeedSum += fSpeed;
	            speedCount++;
	            if (speedCount >= 20) // média a cada 1s
	            {
	                gvTelemetry.speedAverage = fSpeedSum / speedCount;
	                fSpeedSum = 0.0f;
	                speedCount = 0;
	            }
	            osMutexRelease(mutexTelemetryHandle);
	        }

	        // Prepara strings para o LCD
	        snprintf(lcdLine1, 17, "X:%5.2f Y:%5.2f", gvTelemetry.posX, gvTelemetry.posY);
	        snprintf(lcdLine2, 17, "Th:%5.2f V:%4.2f", gvTelemetry.theta, gvTelemetry.speedCurrent);
	        memcpy(lcdBuffer, lcdLine1, 16);
	        memcpy(lcdBuffer+16, lcdLine2, 16);
	        osMessageQueuePut(qLCDDataHandle, lcdBuffer, 0, 0);
	    }

  /* USER CODE END vStartTaskOdometria */
}

/* USER CODE BEGIN Header_vStartTaskUART */
/**
* @brief Function implementing the vTaskUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartTaskUART */
void vStartTaskUART(void *argument)
{
  /* USER CODE BEGIN vStartTaskUART */
  /* Infinite loop */
	    (void)argument;
	    uint8_t ucRxByte;
	    static uint8_t rxBuffer[64];
	    static uint8_t rxIndex = 0;
	    TelemetryData_t telemetrySnap;
	    char txBuffer[256];
	    const TickType_t xTelemetryPeriod = pdMS_TO_TICKS(1000);
	    TickType_t xLastTelemetryTime = xTaskGetTickCount();

	    for(;;)
	    {
	        // Aguarda byte recebido (semáforo dado pela ISR da UART)
	        if (osSemaphoreAcquire(semBTRxReadyHandle, pdMS_TO_TICKS(100)) == osOK)
	        {
	            // Lê o byte do buffer da UART (substitua pela sua implementação)
	            // Exemplo: ucRxByte = uart_rx_buffer[uart_rx_index++];
	            // Por simplicidade, usamos um placeholder. Você deve integrar com a HAL.
	            // Supondo que o byte já esteja disponível em alguma variável global.
	            // Aqui vamos simular a leitura de um buffer circular gerenciado pela ISR.
	            // Se você não tiver, pode usar HAL_UART_Receive_IT e um buffer.
	            // Exemplo fictício:
	            // extern uint8_t uart_rx_byte;
	            // ucRxByte = uart_rx_byte;

	            // Para efeito de compilação, apenas um placeholder:
	            ucRxByte = 0; // <-- substitua pela leitura real

	            if (ucRxByte == '\n' || ucRxByte == '\r')
	            {
	                rxBuffer[rxIndex] = '\0';
	                // Processa comando
	                if (strcmp((char*)rxBuffer, "CALIBRATE") == 0)
	                    vTaskResume(vTaskCalibracaoHandle);
	                else if (strncmp((char*)rxBuffer, "SET_PID ", 8) == 0)
	                {
	                    float kp, ki, kd;
	                    sscanf((char*)rxBuffer+8, "%f %f %f", &kp, &ki, &kd);
	                    if (osMutexAcquire(mutexPIDParamsHandle, pdMS_TO_TICKS(5)) == osOK)
	                    {
	                        gvPIDParams.fKp = kp;
	                        gvPIDParams.fKi = ki;
	                        gvPIDParams.fKd = kd;
	                        osMutexRelease(mutexPIDParamsHandle);
	                    }
	                }
	                else if (strcmp((char*)rxBuffer, "GET_PID") == 0)
	                {
	                    float kp, ki, kd;
	                    if (osMutexAcquire(mutexPIDParamsHandle, pdMS_TO_TICKS(5)) == osOK)
	                    {
	                        kp = gvPIDParams.fKp;
	                        ki = gvPIDParams.fKi;
	                        kd = gvPIDParams.fKd;
	                        osMutexRelease(mutexPIDParamsHandle);
	                    }
	                    snprintf(txBuffer, sizeof(txBuffer), "PID:%.2f %.2f %.2f\r\n", kp, ki, kd);
	                    HAL_UART_Transmit(&hlpuart1, (uint8_t*)txBuffer, strlen(txBuffer), 100);
	                }
	                else if (strcmp((char*)rxBuffer, "MANUAL") == 0)
	                {
	                    uint8_t mode = MODE_MANUAL;
	                    osMessageQueuePut(qTrocaModoHandle, &mode, 0, 0);
	                }
	                else if (strcmp((char*)rxBuffer, "AUTO") == 0)
	                {
	                    uint8_t mode = MODE_AUTONOMOUS;
	                    osMessageQueuePut(qTrocaModoHandle, &mode, 0, 0);
	                }
	                else if (strncmp((char*)rxBuffer, "MOTOR ", 6) == 0)
	                {
	                    float left, right;
	                    sscanf((char*)rxBuffer+6, "%f %f", &left, &right);
	                    MotorCommand_t cmd = {left, right, 2}; // MANUAL
	                    osMessageQueuePut(qMotorCommandHandle, &cmd, 0, 0);
	                }
	                else if (strcmp((char*)rxBuffer, "STOP") == 0)
	                {
	                    MotorCommand_t cmd = {0,0,0};
	                    osMessageQueuePut(qMotorCommandHandle, &cmd, 0, 0);
	                }
	                rxIndex = 0;
	            }
	            else if (rxIndex < sizeof(rxBuffer)-1)
	            {
	                rxBuffer[rxIndex++] = ucRxByte;
	            }
	        }

	        // Envia telemetria a cada 1 segundo
	        if ((xTaskGetTickCount() - xLastTelemetryTime) >= xTelemetryPeriod)
	        {
	            xLastTelemetryTime = xTaskGetTickCount();
	            if (osMutexAcquire(mutexTelemetryHandle, pdMS_TO_TICKS(5)) == osOK)
	            {
	                memcpy(&telemetrySnap, &gvTelemetry, sizeof(TelemetryData_t));
	                osMutexRelease(mutexTelemetryHandle);
	            }
	            snprintf(txBuffer, sizeof(txBuffer),
	                "X=%.2f Y=%.2f Th=%.2f V=%.2f Vavg=%.2f Dist=%.2f Bat=%u%% Mode=%u Calib=%u\r\n",
	                telemetrySnap.posX, telemetrySnap.posY, telemetrySnap.theta,
	                telemetrySnap.speedCurrent, telemetrySnap.speedAverage, telemetrySnap.distTotal,
	                telemetrySnap.batteryPct, telemetrySnap.systemMode, telemetrySnap.calibDone);
	            HAL_UART_Transmit(&hlpuart1, (uint8_t*)txBuffer, strlen(txBuffer), 100);
	        }
	    }

  /* USER CODE END vStartTaskUART */
}

/* USER CODE BEGIN Header_vStartTaskLCD */
/**
* @brief Function implementing the vTaskLCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartTaskLCD */
void vStartTaskLCD(void *argument)
{
  /* USER CODE BEGIN vStartTaskLCD */

	    (void)argument;
	    uint8_t lcdBuffer[32];
	    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_LCD);
	    TickType_t xLastWakeTime = xTaskGetTickCount();

	    for(;;)
	    {
	        vTaskDelayUntil(&xLastWakeTime, xPeriod);
	        if (osMessageQueueGet(qLCDDataHandle, lcdBuffer, NULL, 0) == osOK)
	        {
	            lcdSetCursorPosition(0, 0);
	            lcdPrintStr(lcdBuffer, 16);
	            lcdSetCursorPosition(0, 1);
	            lcdPrintStr(lcdBuffer+16, 16);
	        }
	        else
	        {
	            lcdSetCursorPosition(0, 0);
	            lcdPrintStr((uint8_t*)"Robot Ready     ", 16);
	            lcdSetCursorPosition(0, 1);
	            lcdPrintStr((uint8_t*)"Mode: ?        ", 16);
	        }
	    }

  /* USER CODE END vStartTaskLCD */
}

/* USER CODE BEGIN Header_vStartTaskButtons */
/**
* @brief Function implementing the vTaskButtons thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartTaskButtons */
void vStartTaskButtons(void *argument)
{
  /* USER CODE BEGIN vStartTaskButtons */
  /* Infinite loop */

	    (void)argument;
	    uint32_t ulBits;
	    ButtonEvent_t btnEvent;
	    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_BUTTONS);
	    TickType_t xLastWakeTime = xTaskGetTickCount();
	    static uint32_t lastButtonBits = 0;
	    static uint32_t debounceBits = 0;

	    for(;;)
	    {
	        vTaskDelayUntil(&xLastWakeTime, xPeriod);
	        ulBits = osEventFlagsWait(evButtonsStateHandle, 0x1F, osFlagsWaitAny, xPeriod);
	        if (ulBits == 0) continue;

	        // Debounce simples: verifica se o mesmo padrão se repete por 2 ciclos (100ms)
	        debounceBits = (debounceBits << 5) | (ulBits & 0x1F);
	        if ((debounceBits & 0x1F) == ((debounceBits >> 5) & 0x1F))
	        {
	            uint32_t changed = ulBits ^ lastButtonBits;
	            for (int i = 0; i < 5; i++)
	            {
	                if (changed & (1 << i))
	                {
	                    btnEvent.ucButtonId = i;
	                    btnEvent.ucEventType = (ulBits & (1 << i)) ? 0 : 1; // 0=PRESS, 1=RELEASE
	                    osMessageQueuePut(qButtonsEventHandle, &btnEvent, 0, 0);
	                }
	            }
	            lastButtonBits = ulBits;
	        }
	    }

  /* USER CODE END vStartTaskButtons */
}

/* USER CODE BEGIN Header_vStartTaskTrocarModo */
/**
* @brief Function implementing the vTaskTrocarModo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartTaskTrocarModo */
void vStartTaskTrocarModo(void *argument)
{
  /* USER CODE BEGIN vStartTaskTrocarModo */
	    (void)argument;
	    uint8_t ucMode;
	    static uint8_t currentMode = MODE_MANUAL;

	    for(;;)
	    {
	        if (osMessageQueueGet(qTrocaModoHandle, &ucMode, NULL, portMAX_DELAY) == osOK)
	        {
	            if (ucMode == MODE_AUTONOMOUS && currentMode != MODE_AUTONOMOUS)
	            {
	                vTaskResume(vTaskSegueLinhaHandle);
	                currentMode = MODE_AUTONOMOUS;
	            }
	            else if (ucMode == MODE_MANUAL && currentMode != MODE_MANUAL)
	            {
	                vTaskSuspend(vTaskSegueLinhaHandle);
	                currentMode = MODE_MANUAL;
	            }
	            if (osMutexAcquire(mutexTelemetryHandle, pdMS_TO_TICKS(5)) == osOK)
	            {
	                gvTelemetry.systemMode = currentMode;
	                osMutexRelease(mutexTelemetryHandle);
	            }
	            uint8_t status = (currentMode == MODE_AUTONOMOUS) ? 0x02 : 0x03;
	            osMessageQueuePut(qSystemStatusHandle, &status, 0, 0);
	        }
	    }

  /* USER CODE END vStartTaskTrocarModo */
}

/* USER CODE BEGIN Header_vStartTaskUltrassonicBuzzer */
/**
* @brief Function implementing the vTaskUltraBuzz thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vStartTaskUltrassonicBuzzer */
void vStartTaskUltrassonicBuzzer(void *argument)
{
  /* USER CODE BEGIN vStartTaskUltrassonicBuzzer */
	    (void)argument;
	    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_ULTRASONIC);
	    TickType_t xLastWakeTime = xTaskGetTickCount();
	    MotorCommand_t stopCmd = {0,0,0};
	    float fDistance;

	    for(;;)
	    {
	        vTaskDelayUntil(&xLastWakeTime, xPeriod);

	        // Dispara o trigger do HC-SR04 (pulso de 10us no pino TRIG)
	        // Exemplo (substitua pelos pinos corretos):
	        // HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	        // delay_us(10);
	        // HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	        // Se usar timer para gerar o pulso, configure-o aqui.

	        // Aguarda o semáforo indicando que o eco foi capturado (timeout de 100ms)
	        if (osSemaphoreAcquire(semUltrassonicHandle, pdMS_TO_TICKS(100)) == osOK)
	        {
	            uint32_t ticks = g_ultrasonicEchoTicks;
	            float time_us = (float)ticks; // se timer configurado para 1us/tick
	            fDistance = time_us * 0.017f; // cm (velocidade do som 340m/s)

	            if (fDistance < STOP_DISTANCE_CM)
	            {
	                osMessageQueuePut(qMotorCommandHandle, &stopCmd, 0, 0);
	                // Aciona buzzer com frequência máxima (ex: PWM 2000Hz)
	                // __HAL_TIM_SET_COMPARE(&htimXX, TIM_CHANNEL_YY, 500);
	            }
	            else if (fDistance < 20.0f)
	            {
	                uint16_t freq = (uint16_t)(500.0f + (20.0f - fDistance) * (1500.0f / 15.0f));
	                if (freq > 2000) freq = 2000;
	                // Ajusta PWM do buzzer
	                // __HAL_TIM_SET_COMPARE(&htimXX, TIM_CHANNEL_YY, 500);
	            }
	            else
	            {
	                // Desliga buzzer
	                // __HAL_TIM_SET_COMPARE(&htimXX, TIM_CHANNEL_YY, 0);
	            }
	        }
	    }

  /* USER CODE END vStartTaskUltrassonicBuzzer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

