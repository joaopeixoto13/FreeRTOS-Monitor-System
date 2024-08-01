/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_LEN 1024
#define EOM	'\n'
#define CR	'\r'

#define MESSAGE_TYPE_S				0
#define MESSAGE_TYPE_BLINK			1
#define MESSAGE_TYPE_PUSH_BUTTON	2
#define MESSAGE_TYPE_PIP			3
#define MESSAGE_TYPE_INV			4
#define MESSAGE_ANIMATION_SP		5
#define MESSAGE_ANIMATION_LEDS		6

#define THREAD_FLAG_PUSHBUTTON		(1U<<0)
#define THREAD_FLAG_MSGRECEIVED		(1U<<1)
#define THREAD_FLAG_MSGSENT			(1U<<2)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


/* Struct to encapsulate the data */
typedef struct Message
{
	uint8_t type;				/* Message Type */
	char led;					/* LED */
	uint16_t led_delay;			/* Frequency */
	uint8_t animation;			/* Animation: SerialPort or LEDs */
} Message_t;

// Global variable to handle the led pin
uint16_t led_pin;

// Global variable to handle the led increment
uint16_t led_inc = 0;

/* USER CODE END Variables */
/* Definitions for cli */
osThreadId_t cliHandle;
const osThreadAttr_t cli_attributes = {
  .name = "cli",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledBlink */
osThreadId_t ledBlinkHandle;
const osThreadAttr_t ledBlink_attributes = {
  .name = "ledBlink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pushButton */
osThreadId_t pushButtonHandle;
const osThreadAttr_t pushButton_attributes = {
  .name = "pushButton",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for highPrioTask */
osThreadId_t highPrioTaskHandle;
const osThreadAttr_t highPrioTask_attributes = {
  .name = "highPrioTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for mediumPrioTask */
osThreadId_t mediumPrioTaskHandle;
const osThreadAttr_t mediumPrioTask_attributes = {
  .name = "mediumPrioTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow6,
};
/* Definitions for lowPrioTask */
osThreadId_t lowPrioTaskHandle;
const osThreadAttr_t lowPrioTask_attributes = {
  .name = "lowPrioTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};
/* Definitions for ledQueue */
osMessageQueueId_t ledQueueHandle;
const osMessageQueueAttr_t ledQueue_attributes = {
  .name = "ledQueue"
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};
/* Definitions for myMutex02 */
osMutexId_t myMutex02Handle;
const osMutexAttr_t myMutex02_attributes = {
  .name = "myMutex02"
};
/* Definitions for pipMutex */
osMutexId_t pipMutexHandle;
const osMutexAttr_t pipMutex_attributes = {
  .name = "pipMutex"
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};
/* Definitions for invBinSemaphore */
osSemaphoreId_t invBinSemaphoreHandle;
const osSemaphoreAttr_t invBinSemaphore_attributes = {
  .name = "invBinSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

uint16_t sendSerialPort(UART_HandleTypeDef *huart, char* ptr, uint16_t len);
void serialPortInit(UART_HandleTypeDef *huart);
uint16_t decodeLedName(char led);
void clearLEDs(void);
void printMenuOptions(void);

/* USER CODE END FunctionPrototypes */

void CLI(void *argument);
void LedBlink(void *argument);
void PushButton(void *argument);
void HighPriorityTask(void *argument);
void MediumPriorityTask(void *argument);
void LowPriorityTask(void *argument);
void Callback01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of myMutex01 */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);

  /* creation of myMutex02 */
  myMutex02Handle = osMutexNew(&myMutex02_attributes);

  /* creation of pipMutex */
  pipMutexHandle = osMutexNew(&pipMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem01 */
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

  /* creation of invBinSemaphore */
  invBinSemaphoreHandle = osSemaphoreNew(1, 1, &invBinSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ledQueue */
  ledQueueHandle = osMessageQueueNew (16, sizeof(Message_t), &ledQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of cli */
  cliHandle = osThreadNew(CLI, NULL, &cli_attributes);

  /* creation of ledBlink */
  ledBlinkHandle = osThreadNew(LedBlink, NULL, &ledBlink_attributes);

  /* creation of pushButton */
  pushButtonHandle = osThreadNew(PushButton, NULL, &pushButton_attributes);

  /* creation of highPrioTask */
  highPrioTaskHandle = osThreadNew(HighPriorityTask, NULL, &highPrioTask_attributes);

  /* creation of mediumPrioTask */
  mediumPrioTaskHandle = osThreadNew(MediumPriorityTask, NULL, &mediumPrioTask_attributes);

  /* creation of lowPrioTask */
  lowPrioTaskHandle = osThreadNew(LowPriorityTask, NULL, &lowPrioTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadSuspend(pushButtonHandle);
  osThreadSuspend(ledBlinkHandle);
  osThreadSuspend(highPrioTaskHandle);
  osThreadSuspend(mediumPrioTaskHandle);
  osThreadSuspend(lowPrioTaskHandle);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_CLI */
/**
  * @brief  Function implementing the cli thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CLI */
void CLI(void *argument)
{
  /* USER CODE BEGIN CLI */

	char *inputMessage;
	static const char command_1[] = "1";
	static const char command_2[] = "2";
	static const char command_3[] = "3";
	static const char command_4[] = "4";
	static const char command_5[] = "5";
	static const char command_6[] = "6";
	static const char command_exit[] = "S";
	int menu = 0;
	Message_t msg;

	/* Initialize the UART3 */
	serialPortInit(&huart3);

	/* Print the menu options */
	printMenuOptions();

  /* Infinite loop */
  for(;;)
  {
	  /*
	   * Wait forever until thread message received flag is set
	   * In this case, this function suspends the execution of this currently task until receive the notification
	   * Best option because when these thread flag are already set, the function returns instantly
	   * 45% faster and use less RAM memory, compared to semaphores (version 1), because is directly associated with this thread.
	   */
	  osThreadFlagsWait(THREAD_FLAG_MSGRECEIVED, osFlagsWaitAll, osWaitForever);

	  /* Extract the input message */
	  inputMessage = receiveSerialPort();

	  /* Exit option */
	  if ((memcmp(inputMessage, command_exit, 1) == 0))
	  {
		  /* Update the message Type */
		  msg.type = MESSAGE_TYPE_S;
		  /*
		   * With this approach, we create a communication channel via Queue in which each task suspends itself.
		   * More modular, better for debugging and error fixing, more elegant and
		   * saves a lot of code (we don't have to repeat for each specific task)
		   */
		  osMessageQueuePut(ledQueueHandle, &msg, 0, 10);

		  if (menu == 2) {
			  osMessageQueuePut(ledQueueHandle, &msg, 0, 10);
		  }

		  if (menu == 3 || menu == 4 || menu == 5 || menu == 6) {
			  osMessageQueuePut(ledQueueHandle, &msg, 0, 10);
			  osMessageQueuePut(ledQueueHandle, &msg, 0, 10);
		  }
		  /* Print the menu options */
		  printMenuOptions();
		  menu = 0;

		  /* Clear the user LEDs */
		  clearLEDs();
	  }

	  /* Choose the Menu 1: Blink LED automatically */
	  else if ((memcmp(inputMessage, command_1, 1) == 0) && !menu)
	  {
		  sendSerialPort(&huart3, "Indicate the LED and time (ms)\r\nExample: G500\r\n>", 48);
		  menu = 1;
	  }

	  /* Choose the Menu 2: Blink LED with push button */
	  else if ((memcmp(inputMessage, command_2, 1) == 0) && !menu)
	  {
		  sendSerialPort(&huart3, "Indicate the LED\r\nExample: G\r\n>", 31);
		  menu = 2;
	  }

	  /* Choose the Menu 3, 4, 5 or 6 */
	  else if (((memcmp(inputMessage, command_3, 1) == 0) || (memcmp(inputMessage, command_4, 1) == 0) ||
			   (memcmp(inputMessage, command_5, 1) == 0) || (memcmp(inputMessage, command_6, 1) == 0)) && !menu)
	  {
		  /* Menu 3: Visualize PIP with SerialPort */
		  if ((memcmp(inputMessage, command_3, 1) == 0) && !menu) {
			  msg.type = MESSAGE_TYPE_PIP;
			  msg.animation = MESSAGE_ANIMATION_SP;
			  menu = 3;
		  }
		  /* Menu 4: Visualize PIP with LEDs */
		  else if ((memcmp(inputMessage, command_4, 1) == 0) && !menu) {
			  msg.type = MESSAGE_TYPE_PIP;
			  msg.animation = MESSAGE_ANIMATION_LEDS;
			  menu = 4;
		  }
		  /* Menu 5: Visualize INV with SerialPort */
		  else if ((memcmp(inputMessage, command_5, 1) == 0) && !menu) {
			  msg.type = MESSAGE_TYPE_INV;
			  msg.animation = MESSAGE_ANIMATION_SP;
			  menu = 5;
		  }
		  /* Menu 6: Visualize INV with LEDs */
		  else if ((memcmp(inputMessage, command_6, 1) == 0) && !menu) {
			  msg.type = MESSAGE_TYPE_INV;
			  msg.animation = MESSAGE_ANIMATION_LEDS;
			  menu = 6;
		  }
		  /* Send the respective message via Queue */
		  if (osMessageQueuePut(ledQueueHandle, &msg, 0, 10) != osOK) {
			  sendSerialPort(&huart3, "Error: Could not put item on Queue\r\n", 36);
		  }

		  /* Resume the Low Priority Task to start the show */
		  if (osThreadResume(lowPrioTaskHandle) != osOK) {
			  sendSerialPort(&huart3, "ERROR: The PIP threads not resumed successfully\r\n", 49);
		  }
		  else {
			  sendSerialPort(&huart3, "Visualizing...\r\n", 16);
		  }
	  }

	  /* Menu 1: Choose the LED and time (ms) */
	  else if (menu == 1)
	  {
		  msg.led = inputMessage[0];					/* Extract the led */
		  char* tail = inputMessage + 1;				/* Extract the led_delay */
		  msg.led_delay = atoi(tail);					/* Convert to integer */
		  msg.led_delay = abs(msg.led_delay);			/* Convert to positive */
		  msg.type = MESSAGE_TYPE_BLINK;				/* Set the message type */

		  /* Send the message via Queue */
		  if (osMessageQueuePut(ledQueueHandle, &msg, 0, 10) != osOK) {
			  sendSerialPort(&huart3, "Error: Could not put item on Queue\r\n", 36);
		  }

		  /* Resume the LED_Blink task */
		  if (osThreadResume(ledBlinkHandle) != osOK) {
			  sendSerialPort(&huart3, "ERROR: The LED_Blink thread not resumed successfully\r\n", 54);
		  }

		  else {
			  sendSerialPort(&huart3, "LED is blinking ...\r\nEnter 'S' to exit\r\n>", 41);
		  }
	  }

	  /* Menu 2: Choose the LED */
	  else if (menu == 2)
	  {
		  msg.led = inputMessage[0];			/* Extract the led */
		  msg.type = MESSAGE_TYPE_PUSH_BUTTON;	/* Set the message type */

		  /* Send the message via Queue */
		  if (osMessageQueuePut(ledQueueHandle, &msg, 0, 10) != osOK) {
			  sendSerialPort(&huart3, "Error: Could not put item on Queue\r\n", 36);
		  }
		  /* Resume the PushButton thread */
		  if (osThreadResume(pushButtonHandle) != osOK) {
			  sendSerialPort(&huart3, "ERROR: The PushButton thread not resumed successfully\r\n", 55);
		  }
		  else {
			  sendSerialPort(&huart3, "Press the Push Button (PC13) to increase the LED blink rate or 'S' to exit\r\n>", 77);
		  }
	  }

	  /* Invalid command */
	  else
	  {
		  sendSerialPort(&huart3, "Invalid command!\r\n>", 19);
	  }

	  osDelay(1);
  }

   /* In case we accidentally exit from task loop */
   osThreadTerminate(NULL);

  /* USER CODE END CLI */
}

/* USER CODE BEGIN Header_LedBlink */
/**
* @brief Function implementing the ledBlink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedBlink */
void LedBlink(void *argument)
{
  /* USER CODE BEGIN LedBlink */

	Message_t msg;

  /* Infinite loop */
  for(;;)
  {
	  /* See if there's a message in the Queue (not blocking) */
	  if (osMessageQueueGet(ledQueueHandle, &msg, 0, 0) == osOK)
	  {
		  /* If the message received is Type Blink the LED */
		  if (msg.type == MESSAGE_TYPE_BLINK)
		  {
			  /* Decode the LED pin */
			  led_pin = decodeLedName(msg.led);

			  /* Restart the timer */
			  osTimerStop(myTimer01Handle);

			  /* Start the timer with the corresponding LED period */
			  osTimerStart(myTimer01Handle, msg.led_delay);
		  }

		  /* If the message received is Type_S (Command to suspend the task) */
		  else if (msg.type == MESSAGE_TYPE_S)
		  {
			  /* Stop the Software Timer */
			  osTimerStop(myTimer01Handle);

			  /* Suspend the Task */
			  osThreadSuspend(ledBlinkHandle);
		  }
	  }
    osDelay(1);
  }

  /* In case we accidentally exit from task loop */
  osThreadTerminate(NULL);

  /* USER CODE END LedBlink */
}

/* USER CODE BEGIN Header_PushButton */
/**
* @brief Function implementing the pushButton thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PushButton */
void PushButton(void *argument)
{
  /* USER CODE BEGIN PushButton */

	osThreadFlagsClear(THREAD_FLAG_PUSHBUTTON);
	Message_t msg;

  /* Infinite loop */
  for(;;)
  {
	  /* See if there's a message in the Queue (not blocking) */
	  if (osMessageQueueGet(ledQueueHandle, &msg, 0, 0) == osOK)
	  {
		  /* If the message received is Type Push Button */
		  if (msg.type == MESSAGE_TYPE_PUSH_BUTTON)
		  {
			  /* Update the message */
			  msg.type = MESSAGE_TYPE_BLINK;
			  msg.led_delay = 1000;

			  /* Put the information on Queue */
			  osMessageQueuePut(ledQueueHandle, &msg, 0, 0);

			  /* Resume the Blink Task */
			  osThreadResume(ledBlinkHandle);

			  /* osDelay() to Blink task can receive via Queue the respective message */
			  osDelay(5);
		  }

		  /* If the message received is Type_S (Command to suspend the task) */
		  else if (msg.type == MESSAGE_TYPE_S)
		  {
			  /* Suspend the Task */
			  osThreadSuspend(pushButtonHandle);
		  }
	  }

	  /* If the push button was pressed */
	  if (osThreadFlagsGet() & THREAD_FLAG_PUSHBUTTON)
	  {
		  /* Clear Flag */
		  osThreadFlagsClear(THREAD_FLAG_PUSHBUTTON);

		  /* Update the led delay */
		  msg.led_delay = 1000-led_inc;

		  /* Put the information on Queue */
		  osMessageQueuePut(ledQueueHandle, &msg, 0, 0);

		  /* For debounce */
		  HAL_Delay(200);
	  }

    osDelay(1);
  }

  /* In case we accidentally exit from task loop */
  osThreadTerminate(NULL);

  /* USER CODE END PushButton */
}

/* USER CODE BEGIN Header_HighPriorityTask */
/**
* @brief Function implementing the highPrioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HighPriorityTask */
void HighPriorityTask(void *argument)
{
  /* USER CODE BEGIN HighPriorityTask */
	Message_t msg;
	bool flag = 0;
	bool update = 0;
  /* Infinite loop */
  for(;;)
  {
	  /* See if there's a message in the Queue (not blocking) */
	  if (osMessageQueueGet(ledQueueHandle, &msg, 0, 0) == osOK)
	  {
		  /* If the message received is Type_S (Command to suspend the task) */
		  if (msg.type == MESSAGE_TYPE_S) {
			  osMutexRelease(pipMutexHandle);
			  flag = 0;
			  update = 0;
			  osThreadSuspend(highPrioTaskHandle);
		  }
		  /* If the message received is to start */
		  else {
			  flag = 0;
			  update = 1;
		  }
		  /* osDelay() to other tasks can receive via Queue the respective messages */
		  osDelay(5);
	  }

	  /* Safer, as it only starts when all parameters are configured */
	  if (update)
	  {
		  /* Animation: Serial Port or LEDs */
		  if (msg.animation == MESSAGE_ANIMATION_SP)
			  sendSerialPort(&huart3, "High Task\r\n", 11);
		  else if (msg.animation == MESSAGE_ANIMATION_LEDS)
			  HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);

		  HAL_Delay(250);

		  /* Only for the first iteration */
		  if (!flag) {
			  /* Mechanism: PIP or INV */
			  if (msg.type == MESSAGE_TYPE_PIP)
				  osMutexAcquire(pipMutexHandle, osWaitForever);
			  else if (msg.type == MESSAGE_TYPE_INV)
				  osSemaphoreAcquire(invBinSemaphoreHandle, osWaitForever);
			  flag = 1;
		  }
		  HAL_Delay(250);
	  }
  }
  /* In case we accidentally exit from task loop */
  osThreadTerminate(NULL);

  /* USER CODE END HighPriorityTask */
}

/* USER CODE BEGIN Header_MediumPriorityTask */
/**
* @brief Function implementing the mediumPrioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MediumPriorityTask */
void MediumPriorityTask(void *argument)
{
  /* USER CODE BEGIN MediumPriorityTask */
	Message_t msg;
	msg.animation = MESSAGE_ANIMATION_SP;
  /* Infinite loop */
  for(;;)
  {
	  /* See if there's a message in the Queue (not blocking) */
	  if (osMessageQueueGet(ledQueueHandle, &msg, 0, 0) == osOK)
	  {
		  /* If the message received is Type_S (Command to suspend the task) */
		  if (msg.type == MESSAGE_TYPE_S) {
			  osSemaphoreRelease(invBinSemaphoreHandle);
			  osThreadSuspend(mediumPrioTaskHandle);
		  }
		  /* osDelay() to other tasks can receive via Queue the respective messages */
		  osDelay(5);
	  }
	  else
	  {
		  /* Animation: Serial Port or LEDs */
		  if (msg.animation == MESSAGE_ANIMATION_SP)
			  sendSerialPort(&huart3, "Medium Task\r\n", 13);
		  else if (msg.animation == MESSAGE_ANIMATION_LEDS)
			  HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		  HAL_Delay(500);
	  }

  }
  /* In case we accidentally exit from task loop */
  osThreadTerminate(NULL);

  /* USER CODE END MediumPriorityTask */
}

/* USER CODE BEGIN Header_LowPriorityTask */
/**
* @brief Function implementing the lowPrioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LowPriorityTask */
void LowPriorityTask(void *argument)
{
  /* USER CODE BEGIN LowPriorityTask */
	Message_t msg;
	bool update = 0;
	int try = 0;
  /* Infinite loop */
  for(;;)
  {
	  /* See if there's a message in the Queue (not blocking) */
	  if (osMessageQueueGet(ledQueueHandle, &msg, 0, 0) == osOK)
	  {
		  /* If the message received is Type_S (Command to suspend the task) */
		  if (msg.type == MESSAGE_TYPE_S) {
			  update = 0;
			  try = 0;
			  osThreadSuspend(lowPrioTaskHandle);
		  }
		  /* If the message received is to start */
		  else {
			  try = 0;
			  update = 1;
		  }
		  /* osDelay() to other tasks can receive via Queue the respective messages */
		  osDelay(5);
	  }

	  /* Safer, as it only starts when all parameters are configured */
	  if (update)
	  {
		  /* Animation: Serial Port or LEDs */
		  if (msg.animation == MESSAGE_ANIMATION_SP)
			  sendSerialPort(&huart3, "Low Task\r\n", 10);
		  else if (msg.animation == MESSAGE_ANIMATION_LEDS)
			  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		  HAL_Delay(250);

		  try++;

		  /* For the first iteration */
		  if (try == 1) {
			  /* Mechanism: PIP or INV */
			  if (msg.type == MESSAGE_TYPE_PIP) {
				  osMutexAcquire(pipMutexHandle, 100);
				  sendSerialPort(&huart3, "Mutex Acquired by Low Task\r\n", 28);
			  }
			  else if (msg.type == MESSAGE_TYPE_INV) {
				  osSemaphoreAcquire(invBinSemaphoreHandle, 100);
				  sendSerialPort(&huart3, "Semaphore Acquired by Low Task\r\n", 32);
			  }

			  /* Send the respective information via Queue to other Tasks */
			  osMessageQueuePut(ledQueueHandle, &msg, 0, 0);
			  osMessageQueuePut(ledQueueHandle, &msg, 0, 0);

			  /* Resume the other Tasks */
			  osThreadResume(highPrioTaskHandle);
			  osThreadResume(mediumPrioTaskHandle);

			  try = 1;
		  }

		  /* If the 10th iteration has been reached */
		  if (try == 11) {
			  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
			  if (msg.type == MESSAGE_TYPE_PIP) {
				  sendSerialPort(&huart3, "Mutex Release by Low Task\r\n", 27);
				  osMutexRelease(pipMutexHandle);
			  }
			  else if (msg.type == MESSAGE_TYPE_INV) {
				  sendSerialPort(&huart3, "Semaphore Release by Low Task\r\n", 31);
				  osSemaphoreAcquire(invBinSemaphoreHandle, 100);
			  }
			  try = 0;
		  }

		  HAL_Delay(250);
	  }
  }
  /* In case we accidentally exit from task loop */
  osThreadTerminate(NULL);

  /* USER CODE END LowPriorityTask */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

	/* Toggle the respective LED */
	HAL_GPIO_TogglePin(GPIOB, led_pin);

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// Reception Buffer struct
typedef struct RX_BUFFER
{
	uint8_t buffer[RX_LEN];						/* Buffer */
	uint32_t start, end;						/* Start index & End index */
	uint32_t len;								/* Number of occupied positions */
} rx_buffer_t;

rx_buffer_t Rx_buffer;


char str[RX_LEN];

/*
 * @brief UART Reception Complete Callback
 * @param huart specifies the UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(&huart3, &Rx_buffer.buffer[++Rx_buffer.end & (RX_LEN - 1)], 1);
		Rx_buffer.len++;
		if (Rx_buffer.buffer[(Rx_buffer.end - 1) & (RX_LEN - 1)] == EOM) {
			/* Release the Semaphore */
			osThreadFlagsSet(cliHandle, THREAD_FLAG_MSGRECEIVED);
		}
	}
}

/*
 * @brief UART Transmission Complete Callback
 * @param huart specifies the UART handle
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart->Instance == USART3) {
		/* Release the Tx Complete Semaphore */
		osSemaphoreRelease(myBinarySem01Handle);
	}
}

/*
 * @brief Serial Port Initialization
 * @param huart specifies the UART handle
 * @retval None
 */
void serialPortInit(UART_HandleTypeDef *huart)
{
	Rx_buffer.start = 0;
	Rx_buffer.end = 0;
	Rx_buffer.len = 0;
	HAL_UART_Receive_IT(huart, &Rx_buffer.buffer[Rx_buffer.end & (RX_LEN - 1)], 1);
}


/*
 * @brief Serial Port Send Message
 * @param huart specifies the UART handle
 * @param ptr specifies the message
 * @param len specifies the message length
 * @retval HAL_Status
 */
uint16_t sendSerialPort(UART_HandleTypeDef *huart, char* ptr, uint16_t len)
{
	/* HAL_status flag */
	HAL_StatusTypeDef res;

	/* Protect the communication --> Acquire the Mutex */
	osMutexAcquire(myMutex01Handle, 1000);

	/* Try to acquire the send semaphore */
	if (osSemaphoreAcquire(myBinarySem01Handle, 100) == osOK)
	{
		do
		{
			res = HAL_UART_Transmit_IT(huart, (uint8_t *)ptr, len);
		} while (res == HAL_BUSY);

		if (res == HAL_TIMEOUT)
		{
			len = 0;
			huart->gState == HAL_UART_STATE_READY;
		}
	}

	/* Release the Mutex --> Communication done */
	osMutexRelease(myMutex01Handle);

	return len;
}


/*
 * @brief Serial Port Receive Message
 * @param None
 * @retval Message received
 */
char* receiveSerialPort(void)
{
	int i = 0;

	/* Protect the read operation --> Acquire the Mutex */
	osMutexAcquire(myMutex02Handle, 1000);

	while (Rx_buffer.buffer[Rx_buffer.start & (RX_LEN - 1)] != CR)
	{
		str[i++] = Rx_buffer.buffer[Rx_buffer.start++ & (RX_LEN - 1)];
		Rx_buffer.len--;
	}
	str[i] = '\0';
	Rx_buffer.start++;
	Rx_buffer.start++;
	Rx_buffer.len--;
	Rx_buffer.len--;

	/* Release the Mutex */
	osMutexRelease(myMutex02Handle);

	if(str[0] == '\0')
		return "\n";

	return str;
}

/*
 * @brief External GPIO callback
 * @param None
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13) {
		/* Set the respective thread flag */
		osThreadFlagsSet(pushButtonHandle, THREAD_FLAG_PUSHBUTTON);

		/* Increment the led inc */
		led_inc = (led_inc + 100) % 1000;
	}
}


/*
 * @brief Decode the led name
 * @param Led name ('G', 'B' or 'R')
 * @retval GPIO_PIN_X
 */
uint16_t decodeLedName(char led)
{
	uint16_t led_pin;

	if (led == 'B') {
		led_pin = GPIO_PIN_7;
	}
	else if (led == 'R') {
		led_pin = GPIO_PIN_14;
	}
	else  {
		led_pin = GPIO_PIN_0;
	}
	return led_pin;
}


/*
 * @brief Clear the user LEDs
 * @param None
 * @retval None
 */
void clearLEDs(void)
{
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);		/* Clear Green LED */
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);		/* Clear Blue LED */
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);			/* Clear Red LED */
}


void printMenuOptions(void)
{
	sendSerialPort(&huart3, "Welcome to Real Time Operating Systems!\r\n", 41);
	sendSerialPort(&huart3, "1: Blink a LED automatically\r\n", 30);
	sendSerialPort(&huart3, "2: Blink a LED with a push button\r\n", 35);
	sendSerialPort(&huart3, "3: Visualize the PIP with SerialPort\r\n", 38);
	sendSerialPort(&huart3, "4: Visualize the PIP with LEDs\r\n", 32);
	sendSerialPort(&huart3, "5: Visualize the Semaphore Priority Inversion with SerialPort\r\n", 63);
	sendSerialPort(&huart3, "6: Visualize the Semaphore Priority Inversion with LEDs\r\n>", 58);
}

/* USER CODE END Application */

