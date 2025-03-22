/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2021 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define TRUE 1
#define FALSE 0

/* USER CODE BEGIN Variables */

/*Prioridades de las Tareas Periodicas*/
#define PR_TAREA1 1
#define PR_TAREA2 2
#define PR_TAREA3 3
#define PR_TAREA4 4
#define PR_TAREA5 5
#define PR_TAREA6 6
/*Periodos de las tareas*/
#define T_TAREA1 200
#define T_TAREA2 300
#define T_TAREA3 300
#define T_TAREA4 200
#define T_TAREA5 400
#define T_TAREA6 200
#define LONG_TIME 0xffff

 
 /* USER CODE BEGIN RTOS_MUTEX */
 
/*********************Semaforo 1*********************/ ;
SemaphoreHandle_t Semaforo_1 = NULL;
SemaphoreHandle_t xSemaphore = NULL;

// Semaforo para sincronizar tarea esporadica e interrupcion
SemaphoreHandle_t Semaforo_Interrupcion = NULL; 
// Crear el semaforo binario 



/* Declarar aqui las variables protegidas por el semaforo 1 */
double Altitud = 0;	// Altitud  
double RX = 0;		// Inclinacion eje X 
double RY = 0;		// Inclinacion eje Y
double X,Y,Z;
double lastaltitud = 0;
double lastX = 0;
double lastY =  0;
double debugX =  0;
int vibraciones = 1;
int AltitudMotor = 0, RyMotor = 0 , RxMotor = 0;
int potenciaMotor = 0;

/* Tarea y mutex creados con CMSIS 
osThreadId Tarea1Handle;
osMutexId mutex1Handle; */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartTarea1(void const * argument);
void startTarea2(void const * argument);
void startTarea3(void const * argument);
void startTarea4(void const * argument);
void startTarea5(void const * argument);
void startTarea6(void const * argument);
void startTarea7(void const * argument);
int checkVibration();
/* Variables para depuracion */
int ContTarea1 = 0;
int altitud_objetivo = 600;
int start= 0;




/* Calculate acc. coordinates and stores them in global variables X Y Z */
void Obtain_Coordinates_XYZ(){
	int Ix, Iy, Iz;
	uint8_t Ix1, Ix2;
	uint8_t Iy1, Iy2;
	uint8_t Iz1, Iz2;
	Ix1 = SPI_Read (0x28);
	Ix2 = SPI_Read (0x29);
	Ix = (Ix2 << 8) + Ix1;
	if (Ix >= 0x8000) Ix = -(65536 - Ix);
	X = Ix/16384.0;
	Iy1 = SPI_Read (0x2A);
	Iy2 = SPI_Read (0x2B);
	Iy = (Iy2 << 8) + Iy1;
	if (Iy >= 0x8000) Iy = -(65536 - Iy);
	Y = Iy/16384.0;
	Iz1 = SPI_Read (0x2C);
	Iz2 = SPI_Read (0x2D);
	Iz = (Iz2 << 8) + Iz1;
	if (Iz >= 0x8000) Iz = -(65536 - Iz);
	Z = Iz/16384.0;
}

double Calculate_RotationX (){ 
	double rotX;
	Obtain_Coordinates_XYZ();
	rotX = atan2(Y, sqrt(X*X+Z*Z)) * 180.0/3.1416;
	return rotX;
}

double Calculate_RotationY (){
	double rotY;
	Obtain_Coordinates_XYZ();
	rotY = - atan2(X, sqrt(Y*Y+Z*Z)) * 180.0/3.1416;
	return rotY;
}


void Potencia(int potencia){
	switch (potencia) {
	case 5:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
		break;
	case 0:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
		break;
	}
}

void M1on(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
}
void M1off(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
}
void M2on(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
}
void M2off(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}
void M3on(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}
void M3off(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}
void M4on(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}
void M4off(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
}

void AllMon(void){
	M1on();
	M2on();
	M3on();
	M4on();
}

void AllMoff(void){
	M1off();
	M2off();
	M3off();
	M4off();
}
void L1on(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
}
void L1off(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
}
void L2on(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}
void L2off(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

enum estado_t {STANDBY, INCL_OK, ADJUSTING, ALTITUD ,STOP};
enum estado_t estado = STANDBY;

void Xfor(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = ADJUSTING;
		M1on();
		M2off();
		break;
	case ADJUSTING:
		estado = ADJUSTING;
		M1on();
		M2off();
		break;
	case ALTITUD:
		estado = ADJUSTING;
		M1on();
		M2off();
		break;
	case STOP:
		estado = STOP;
		break;
	}
	xSemaphoreGive(xSemaphore);
}

void Xback(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = ADJUSTING;
		M2on();
		M1off();
		break;
	case ADJUSTING:
		estado = ADJUSTING;
		M2on();
		M1off();
		break;
	case ALTITUD:
		estado = ADJUSTING;
		M2on();
		M1off();
		break;
	case STOP:
		estado = STOP;
		break;
	}
	xSemaphoreGive(xSemaphore);
}


void X0(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = INCL_OK;
		M2off();
		M1off();
		break;
	case ADJUSTING:
		estado = INCL_OK;
		M2off();
		M1off();
		break;
	case ALTITUD:
		estado = ALTITUD;
		break;
	case STOP:
		estado = STOP;
		break;
	}
	xSemaphoreGive(xSemaphore);
}

void Yfor(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = ADJUSTING;
		M3on();
		M4off();
		break;
	case ADJUSTING:
		estado = ADJUSTING;
		M3on();
		M4off();
		break;
	case ALTITUD:
		estado = ADJUSTING;
		M3on();
		M4off();
		break;
	case STOP:
		estado = STOP;
		break;
	}
	xSemaphoreGive(xSemaphore);
}

void Yback(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = ADJUSTING;
		M4on();
		M3off();
		break;
	case ADJUSTING:
		estado = ADJUSTING;
		M4on();
		M3off();
		break;
	case ALTITUD:
		estado = ADJUSTING;
		M4on();
		M3off();
		break;
	case STOP:
		estado = STOP;
		break;
	}
	xSemaphoreGive(xSemaphore);
}


void Y0(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = INCL_OK;
		M4off();
		M3off();
		break;
	case ADJUSTING:
		estado = INCL_OK;
		M4off();
		M3off();
		break;
	case ALTITUD:
		estado = ALTITUD;
		break;
	case STOP:
		estado = STOP;
		break;
	}
	xSemaphoreGive(xSemaphore);
}

void Altok(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = INCL_OK;
		break;
	case ADJUSTING:
		estado = ADJUSTING;
		break;
	case ALTITUD:
		estado = INCL_OK;
		M1off();
		M2off();
		M4off();
		M3off();
		break;
	case STOP:
		estado = STOP;
		break;
	}
	xSemaphoreGive(xSemaphore);
}

void Ascend(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = ALTITUD;
		M1on();
		M2on();
		M3on();
		M4on();
		break;
	case ALTITUD:
		estado = ALTITUD;
		M1on();
		M2on();
		M3on();
		M4on();
		break;
	case ADJUSTING:
		estado = ADJUSTING;
		break;
	case STOP:
		estado = STOP;
		break;
	}
	xSemaphoreGive(xSemaphore);
}

void Risk(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = STOP;
		M1off();
		M2off();
		M4off();
		M3off();
		L2on();
		L1off();
		break;
	case ALTITUD:
		estado = STOP;
		M1off();
		M2off();
		M4off();
		M3off();
		L2on();
		L1off();
		break;
	case ADJUSTING:
		estado = STOP;
		M1off();
		M2off();
		M4off();
		M3off();
		L2on();
		L1off();
		break;
	case STOP:
		estado = STOP;
		M1off();
		M2off();
		M4off();
		M3off();
		L2on();
		L1off();
		break;
	}
	xSemaphoreGive(xSemaphore);
}

void Recover(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = INCL_OK;
		break;
	case ALTITUD:
		estado = ALTITUD;
		break;
	case ADJUSTING:
		estado = ADJUSTING;
		break;
	case STOP:
		estado = STANDBY;
		L2off();
		break;
	}
	xSemaphoreGive(xSemaphore);
}

void Wait(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		estado = STANDBY;
		break;
	case INCL_OK:
		estado = STANDBY;
		M1off();
		M2off();
		M4off();
		M3off();
		L1off();
		start=0;
		break;
	case ALTITUD:
		estado = STANDBY;
		M1off();
		M2off();
		M4off();
		M3off();
		L1off();
		start=0;
		break;
	case ADJUSTING:
		estado = STANDBY;
		M1off();
		M2off();
		M4off();
		M3off();
		L1off();
		start=0;
		break;
	case STOP:
		estado = STOP;
		break;
	}
	xSemaphoreGive(xSemaphore);
}

void Start(void){
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	switch(estado){
	case STANDBY:
		start = 1;
		estado = INCL_OK;
		L1on();
		break;
	case INCL_OK:
		estado = ADJUSTING;
		break;
	case ALTITUD:
		estado = ALTITUD;
		break;
	case ADJUSTING:
		estado = ADJUSTING;
		break;
	case STOP:
		estado = STOP;
		break;
	}
	xSemaphoreGive(xSemaphore);
}




/* USER CODE END FunctionPrototypes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_CAN1_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
	MX_I2C2_Init();
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
	/* USER CODE BEGIN Init */
	Inicializa_Acelerometro();
	/* USER CODE END Init */
	xSemaphore = xSemaphoreCreateMutex();
	Semaforo_Interrupcion = xSemaphoreCreateBinary();
	/* Create the mutex using CMSIS
	osMutexDef(mutex1);
	mutex1Handle = osMutexCreate(osMutex(mutex1)); */

	/* Create the thread using CMSIS
	osThreadDef(Tarea1, StartTarea1, osPriorityNormal, 0, 128);
	Tarea1Handle = osThreadCreate(osThread(Tarea1), NULL); */

	/* USER CODE BEGIN RTOS_THREADS using FreeRTOS */
	xTaskCreate(StartTarea1, "Tarea inicial", configMINIMAL_STACK_SIZE, NULL, PR_TAREA1, NULL);
	xTaskCreate(startTarea2, "Tarea inicial 2", configMINIMAL_STACK_SIZE, NULL, PR_TAREA2, NULL);
	xTaskCreate(startTarea3, "Tarea inicial 3", configMINIMAL_STACK_SIZE, NULL, PR_TAREA3, NULL); 
	xTaskCreate(startTarea4, "Tarea inicial 4", configMINIMAL_STACK_SIZE, NULL, PR_TAREA4, NULL);
	xTaskCreate(startTarea5, "Tarea inicial 5", configMINIMAL_STACK_SIZE, NULL, PR_TAREA5, NULL);
	xTaskCreate(startTarea7, "Tarea inicial 7", configMINIMAL_STACK_SIZE, NULL, PR_TAREA6, NULL);
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	while (1) {
	}
	return 0;
}   /* main END */


/* USER CODE Start */


// t1-Altitude
void StartTarea1(void const * argument){
	TickType_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount();
	for(;;){
		ContTarea1 ++;
		RX = Calculate_RotationX();
		RY = Calculate_RotationY();
		// osDelay(1);
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(T_TAREA1));
	}
}


// t2-Inclin
void startTarea2(void const * argument){
	TickType_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount();
	int Lectura_ADC1 = 0; //Valor leido
	/* Inicializa canal 1 del ADC1 */
	ADC_ChannelConfTypeDef sConfigN = {0}; // Variable local en la tarea
	sConfigN.Channel = ADC_CHANNEL_1; // selecciona el canal 1
	sConfigN.Rank = 1;
	sConfigN.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfigN); // configura ADC1-Canal_1
	for(;;){
		// Activacion de la lectura
		HAL_ADC_Start(&hadc1); // comienza la converson AD
		if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
			Lectura_ADC1 = HAL_ADC_GetValue(&hadc1); // leemos el valor
			Altitud = Lectura_ADC1; // actualizamos una variable global }
			vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(T_TAREA2)); 
		}
	}
}



// t3-Vibration
void startTarea3(void const * argument){
	TickType_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount();
	/* 8 y 9 X  10 y 11 para Y  */
	for(;;){
		if (Altitud < altitud_objetivo){
			/* HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); 

			potencia(5*floor(1-(Altitud/ altitud_objetivo)))*/
			Ascend();
			Potencia(floor(5*(1-(Altitud/ altitud_objetivo))));
		}else{
			/*HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); */
			Altok();
		}
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(T_TAREA2)); 
	}

}



// t4-Activate
void startTarea4(void const * argument){
	TickType_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount();
	for(;;){
		if (RX < -10.0){
			Xback();
		}
		else if(RX > 10.0){
			Xfor();
		}
		else{
			X0();
		}

		if (RY < -10.0){
			Yback();
		}
		else if(RY > 10.0){
			Yfor();
		}
		else{
			Y0();
		}

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(T_TAREA4));
	}
}



// t5-Motor Control
void startTarea5(void const * argument){
	TickType_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount();
	for(;;){
		// Obtain_Coordinates_XYZ();
		debugX=X - lastX;
		if(X - lastX > 0.2 || X - lastX < -0.2 || Y - lastY > 0.2 || Y - lastY < -0.2 || Z - lastaltitud > 0.2 || Z - lastaltitud < -0.2){
			vibraciones = vibraciones + 1;
			if(vibraciones == 3){
				Risk();
			}
		}
		else{
			vibraciones = 0;
		}
		lastaltitud = Z;
		lastX = X ;
		lastY =  Y ;
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(T_TAREA5)); 
	}
}



void startTarea7(void const * argument){
	TickType_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount();

	for(;;){
		xSemaphoreTake(Semaforo_Interrupcion, portMAX_DELAY);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
		if(start == 0){Start();}else {Wait();}
	}

}



// Interrupcion de inicio
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	long yield = pdFALSE; 
	// se pondra a pdTRUE si una tarea de mayor prioridad estaba esperando y es desbloqueada
	if (GPIO_Pin == GPIO_PIN_0) {
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Enciende/apaga un LED 
		xSemaphoreGiveFromISR(Semaforo_Interrupcion, &yield);
		portYIELD_FROM_ISR(yield); 
		// para que FreeRTOS haga un cambio de contexto al terminar la ISR
	}
	if (GPIO_Pin == GPIO_PIN_3) {
		//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15); // Enciende/apaga un LED
		xSemaphoreGiveFromISR(Semaforo_Interrupcion, &yield);
		portYIELD_FROM_ISR(yield); 
		// para que FreeRTOS haga un cambio de contexto al terminar la ISR
	}
}



/* USER CODE End */


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
