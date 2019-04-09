/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//Timer
static __IO uint32_t TimingDelay;

//serwomechanizm
volatile uint16_t PomiarADC; //zmienna sluzaca do pomiaru wartosci ADC
uint16_t ADCval; //wartosc ADC potrzebna do sterowania serwem
int move; //Zmienna przechowuj1ca wartosc wypelnienia
uint16_t a = 0; //dolny zakres pracy joysticka (dolna wartosc ADC)
uint16_t b = 4095; //gorny zakres pracy joysticka (gorna wartosc ADC)

//bufor kolowy - odczyt
char str[256];		//tablica charow do transmisji w terminalu
char bufor[256];	//bufor kolowy
int bufor_e = 0;	//pierwszy wolny indeks
int bufor_f = 0;	//kolejny zajety indeks

//bufor kolowy - wysylanie
char bufor2[256]; //bufor kolowy
char pomoc[256];  //tablica charow do transmisji w terminalu
int bufor2_e = 0; //pierwszy wolny indeks
int bufor2_f = 0; //kolejny zajety indeks
int wysylanie; //flaga wysylania 0-brak transmisji 1-transmisja
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
char UART_KB_HIT(); //dopisanie znaku
char UART_GET_CHAR(); //pobranie znaku
char UART_GET_LINE(char chr, char* str); //pobranie linii
void UART_SendData(unsigned char *data, uint16_t len); //wysy3anie

//UART - zabezpiecznia
char * findCommand(char * src, const char * findingCommand); //wyszukiwanie komend
int countCommands(char * src, const char * findingCommand);  //zliczanie komend

//UART - liczniki polecen
int counterPing = 0; //licznik ping;
int counterPomiar = 0; //licznik pomiar;
int counterDefault = 0; //licznik default;
int counterAplus1 = 0; //licznik aplus1;
int counterAplus10 = 0; //licznik aplus10;
int counterAplus100 = 0; //licznik aplus100;
int counterAminus1 = 0; //licznik aminus1;
int counterAminus10 = 0; //licznik aminus10;
int counterAminus100 = 0; //licznik aminus100;
int counterBplus1 = 0; //licznik bplus1;
int counterBplus10 = 0; //licznik bplus10;
int counterBplus100 = 0; //licznik bplus100;
int counterBminus1 = 0; //licznik bminus1;
int counterBminus10 = 0; //licznik bminus10;
int counterBminus100 = 0; //licznik bminus100;

//przerwania
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); //odbior
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart); //transmisja
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc); //pomiar ADC

//protocol commands
void commandPing();		    //ping
void commandPomiar();	    //Pomiar napiecia
void commandDefault();      //Kalibracja joysticka w zakresie 0-3.3V
void commandAplus1();       //zakres dolny +1
void commandAplus10();      //zakres dolny +10
void commandAplus100();     //zakres dolny +100
void commandAminus1();      //zakres dolny -1
void commandAminus10();     //zakres dolny -10
void commandAminus100();    //zakres dolny -100
void commandBplus1();       //zakres gorny +1
void commandBplus10();      //zakres gorny +10
void commandBplus100();     //zakres gorny +100
void commandBminus1();      //zakres gorny -1
void commandBminus10();     //zakres gorny -10
void commandBminus100();    //zakres gorny -100
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	char getchr; //pobrany znak
	//Systick
	if (SysTick_Config(SystemCoreClock / 1000))
		while (1)
			;
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); //generowanie wartosci wypelnienia PWM, potrzebnego do sterowania serwem.
	HAL_ADC_Start(&hadc1); //pomiar adc po stronie joysticka
	HAL_UART_Receive_IT(&huart2, &bufor[bufor_e], 1); //przygotowanie do odbioru znaku
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//Serwo
		ADCval = HAL_ADC_GetValue(&hadc1); //odczytanie wartosci adc
		if (ADCval >= a && ADCval <= b) {
			move = round(60 + ADCval * (190.0 / 4095.0)); //wyliczenie stukrotnosci czasu reakcji w ms. (W lewo 0V, w prawo 3,3V)
			TIM4->CCR2 = move; //Zapisanie wartooci napiecia serwa do rejestru CCR2
			HAL_ADC_Start(&hadc1); // rozpoczecie pomiaru
			Delay(1);
		}
		//UART
		if (UART_KB_HIT() == 1) { //przy wykryciu wprowadzenia znaku wchodzi do petli
			getchr = UART_GET_CHAR(); //pobiera wpisany znak do zmiennej getchr
			int len = UART_GET_LINE(getchr, str); //zapis d3ugosci polecenia z pobranej linii
			if (len > 0) { //jesli jest cokolwiek do odczytania to wejdzie to petli
				//tutaj wejdzie dopiero jak wcisniemy enter
				//komenda pobierana funkcj1 UART_GET_LINE() i sprawdzana pod wzgledem d3ugosci, nazwy komendy oraz srednika na koncu polecenia
				for (int i = 0; str[i]; i++) {
					str[i] = tolower(str[i]);
				}
				if ((countCommands(str, "ping;") > 0)
						|| (countCommands(str, "pomiar;") > 0)
						|| (countCommands(str, "default;") > 0)
						|| (countCommands(str, "aplus1;") > 0)
						|| (countCommands(str, "aplus10;") > 0)
						|| (countCommands(str, "aplus100;") > 0)
						|| (countCommands(str, "aminus1;") > 0)
						|| (countCommands(str, "aminus10;") > 0)
						|| (countCommands(str, "aminus100;") > 0)
						|| (countCommands(str, "bplus1;") > 0)
						|| (countCommands(str, "bplus10;") > 0)
						|| (countCommands(str, "bplus100;") > 0)
						|| (countCommands(str, "bminus1;") > 0)
						|| (countCommands(str, "bminus10;") > 0)
						|| (countCommands(str, "bminus100;") > 0)) {
					counterPing = countCommands(str, "ping;");
					counterPomiar = countCommands(str, "pomiar;");
					counterDefault = countCommands(str, "default;");
					counterAplus1 = countCommands(str, "aplus1;");
					counterAplus10 = countCommands(str, "aplus10;");
					counterAplus100 = countCommands(str, "aplus100;");
					counterAminus1 = countCommands(str, "aminus1;");
					counterAminus10 = countCommands(str, "aminus10;");
					counterAminus100 = countCommands(str, "aminus100;");
					counterBplus1 = countCommands(str, "bplus1;");
					counterBplus10 = countCommands(str, "bplus10;");
					counterBplus100 = countCommands(str, "bplus100;");
					counterBminus1 = countCommands(str, "bminus1;");
					counterBminus10 = countCommands(str, "bminus10;");
					counterBminus100 = countCommands(str, "bminus100;");
					if (counterPing != 0) {
						for (int i = 0; i < counterPing; i++) {
							commandPing();
						}				//for ping;
						counterPing = 0;
					}				//counterPing
					if (counterPomiar != 0) {
						for (int i = 0; i < counterPomiar; i++) {
							commandPomiar();
						}				//for pomiar;
						counterPomiar = 0;
					}				//counterPomiar
					if (counterDefault != 0) {
						for (int i = 0; i < counterDefault; i++) {
							commandDefault();
						}				//for default;
						counterDefault = 0;
					}				//counterDefault
					if (counterAplus1 != 0) {
						for (int i = 0; i < counterAplus1; i++) {
							commandAplus1();
						}				//for Aplus1;
						counterAplus1 = 0;
					}				//counterAplus1
					if (counterAplus10 != 0) {
						for (int i = 0; i < counterAplus10; i++) {
							commandAplus10();
						}				//for Aplus10;
						counterAplus10 = 0;
					}				//counterAplus10
					if (counterAplus100 != 0) {
						for (int i = 0; i < counterAplus100; i++) {
							commandAplus100();
						}				//for Aplus100;
						counterAplus100 = 0;
					}				//counterAplus100
					if (counterAminus1 != 0) {
						for (int i = 0; i < counterAminus1; i++) {
							commandAminus1();
						}				//for Aminus1;
						counterAminus1 = 0;
					}				//counterAminus1
					if (counterAminus10 != 0) {
						for (int i = 0; i < counterAminus10; i++) {
							commandAminus10();
						}				//for Aminus10;
						counterAminus10 = 0;
					}				//counterAminus10
					if (counterAminus100 != 0) {
						for (int i = 0; i < counterAminus100; i++) {
							commandAminus100();
						}				//for Aminus100;
						counterAminus100 = 0;
					}				//counterAminus100
					if (counterBplus1 != 0) {
						for (int i = 0; i < counterBplus1; i++) {
							commandBplus1();
						}				//for Bplus1;
						counterBplus1 = 0;
					}				//counterBplus1
					if (counterBplus10 != 0) {
						for (int i = 0; i < counterBplus10; i++) {
							commandBplus10();
						}				//for Bplus10;
						counterBplus10 = 0;
					}				//counterBplus10
					if (counterBplus100 != 0) {
						for (int i = 0; i < counterBplus100; i++) {
							commandBplus100();
						}				//for Bplus100;
						counterBplus100 = 0;
					}				//counterBplus100
					if (counterBminus1 != 0) {
						for (int i = 0; i < counterBminus1; i++) {
							commandBminus1();
						}				//for Bminus1;
						counterBminus1 = 0;
					}				//counterBminus1
					if (counterBminus10 != 0) {
						for (int i = 0; i < counterBminus10; i++) {
							commandBminus10();
						}				//for Bminus10;
						counterBminus10 = 0;
					}				//counterBminus10
					if (counterBminus100 != 0) {
						for (int i = 0; i < counterBminus100; i++) {
							commandBminus100();
						}				//for Bminus100;
						counterBminus100 = 0;
					}				//counterBminus100
				}				//zliczanie komend
				else {
					errorMessage();
				}				//error
			} //if (len>0)
		} // dopisanie znaku
	} //while
} //main
/* USER CODE END 3 */

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 999;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

}

/* USER CODE BEGIN 4 */
//Komendy glowne
void commandPing() { //test polaczenia
	unsigned char data[256];
	uint16_t size =
			sprintf(data,
					"Jesli widzisz ten komunikat, to znaczy, ze poprawnie polaczyles sie z terminalem.\r\n");
	UART_SendData(data, size);
}

void commandPomiar() { //pomiar napiecia
	unsigned char data[256];
	PomiarADC = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Start(&hadc1);
	uint16_t size = sprintf(data, "ADC = %d (%.3fV)\r\n", PomiarADC,
			PomiarADC * 3.3f / 4096.0f);
	UART_SendData(data, size);
}

void commandDefault() { //ustawienie domyslenego zakresu pracy
	unsigned char data[256];
	uint16_t size = sprintf(data, "Tryb pracy w zakresie 0-3.3V\r\n");
	UART_SendData(data, size);
	a = 0;
	b = 4095;
}

void errorMessage() { //wydruk bledu
	unsigned char data[256];
	uint16_t size =
			sprintf(data,
					"Blad! Dozwolone komendy: 'ping;', 'pomiar;', 'default;', 'aplus1;', 'aplus10;', 'aplus100;', 'aminus1;', 'aminus10;', 'aminus100;', 'bplus1;', 'bplus10;', 'bplus100;', 'bminus1;', 'bminus10;', 'bminus100;'\r\n");
	UART_SendData(data, size);
}

//Przerwania
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { //przerwanie UART - odbior
	if (huart->Instance == USART2) {
		bufor_e++;
		if (bufor_e >= 256) {
			bufor_e = 0;
		}
		HAL_UART_Receive_IT(&huart2, &bufor[bufor_e], 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { //przerwanie UART - wysylanie
	if (huart->Instance == USART2) {
		if (bufor2_e != bufor2_f) {
			int i = 0;
			while (bufor2_e != bufor2_f) {
				pomoc[i] = bufor2[bufor2_f];
				bufor2_f++;
				i++;
			}
			HAL_UART_Transmit_IT(&huart2, pomoc, i);
			if (bufor2_f >= 256) {
				bufor2_f = 0;
			}
		} else {
			wysylanie = 0;
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	PomiarADC = HAL_ADC_GetValue(&hadc1);
}

//UART
char UART_KB_HIT() {	//wykrywanie dopisanina znaku do bufora
	if (bufor_e == bufor_f) {
		return 0;
	} else {
		return 1;
	}
}

char UART_GET_CHAR() { //odczytywanie znaku z bufora
	char x;
	x = bufor[bufor_f];
	bufor_f++;
	if (bufor_f >= 256) {
		bufor_f = 0;
	}
	return x;
}

char UART_GET_LINE(char chr, char* str) { //pobieranie linii z terminala
	static char mem[256]; //bufor odczytu
	static int i = 0;
	int j;
	if (chr == 10 || chr == 13) { //10 lub 13 oznacza znak konca linii (byc moze oba)
		memcpy(str, mem, i); //kopiowanie zawartosci mem do str (ilosc charow definiuje zmienna i)
		str[i] = 0;
		j = i;
		i = 0;
		return j;
	} else {
		mem[i] = chr; //zapis znaku do bufora odczytu
		i++;
		if (i >= 256) {
			i = 0;
		}
		return 0;
	}
}

void UART_SendData(unsigned char *data, uint16_t len) {
	for (int i = 0; i < len; i++) {
		bufor2[bufor2_e] = data[i];
		bufor2_e++;
		if (bufor2_e >= 256) {
			bufor2_e = 0;
		}
	}
	__disable_irq();
	if (wysylanie == 0) {
		wysylanie = 1;
		int i = 0;
		while (bufor2_f != bufor2_e) {
			pomoc[i] = bufor2[bufor2_f];
			bufor2_f++;
			if (bufor2_f >= 256) {
				bufor2_f = 0;
			}
			i++;
		}
		HAL_UART_Transmit_IT(&huart2, pomoc, i);
	}
	__enable_irq();
}

//zabezpieczenia

char * findCommand(char * src, const char * findingCommand) {
	const char * beginingOfsearchingCommand = findingCommand;
	char * currPositionOfSource = src;

	while (*src) {
		findingCommand = beginingOfsearchingCommand;
		src = currPositionOfSource;

		while (*src == *findingCommand) {
			if ((*src == '\0' && *findingCommand == '\0')
					|| (*src == ';' && *findingCommand == ';'))
				break;

			src++;
			findingCommand++;
		}

		if (*findingCommand == '\0' || *findingCommand == ';')
			if (*src == ' ' || *src == '\0' || *src == ';')
				return currPositionOfSource;

		currPositionOfSource++;
	}

	return NULL;
}

int countCommands(char * src, const char * findingCommand) {
	char * placeInString = findCommand(src, findingCommand);
	int mainCounter = 0;

	while (placeInString) {
		mainCounter++;
		placeInString = findCommand(placeInString + 1, findingCommand);
	}

	return mainCounter;
}

//Opoznienie
void Delay(__IO uint32_t nTime) {
	TimingDelay = nTime;

	while (TimingDelay != 0)
		;
}

void TimingDelay_Decrement(void) {
	if (TimingDelay != 0x00) {
		TimingDelay--;
	}
}

//Ustawienie czulosci
void commandAplus1() {
	if ((a + 1) < b) {
		a = a + 1;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres dolny nie mo?e przekraczac %d!\r\n", b - 1);
		UART_SendData(data, size);
	}
} //zakres dolny +1
void commandAplus10() {
	if ((a + 10) < b) {
		a = a + 10;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres dolny nie mo?e przekraczac %d!\r\n", b - 1);
		UART_SendData(data, size);
	}
}      //zakres dolny +10
void commandAplus100() {
	if ((a + 100) < b) {
		a = a + 100;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres dolny nie mo?e przekraczac %d!\r\n", b - 1);
		UART_SendData(data, size);
	}
}     //zakres dolny +100
void commandAminus1() {
	if ((a - 1) >= 0) {
		a = a - 1;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres dolny nie mo?e przekraczac 0!\r\n");
		UART_SendData(data, size);
	}
}      //zakres dolny -1
void commandAminus10() {
	if ((a - 10) >= 0) {
		a = a - 10;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres dolny nie mo?e przekraczac 0!\r\n");
		UART_SendData(data, size);
	}
}     //zakres dolny -10

void commandAminus100() {
	if ((a - 100) >= 0) {
		a = a - 100;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres dolny nie mo?e przekraczac 0!\r\n");
		UART_SendData(data, size);
	}
}    //zakres dolny -100
void commandBplus1() {
	if ((b + 1) < 4096) {
		b = b + 1;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres gorny nie mo?e przekraczac 4095!\r\n");
		UART_SendData(data, size);
	}
}       //zakres gorny +1
void commandBplus10() {
	if ((b + 10) < 4096) {
		b = b + 10;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres gorny nie mo?e przekraczac 4095!\r\n");
		UART_SendData(data, size);
	}
}      //zakres gorny +10
void commandBplus100() {
	if ((b + 100) < 4096) {
		b = b + 100;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres gorny nie mo?e przekraczac 4095!\r\n");
		UART_SendData(data, size);
	}
}     //zakres gorny +100
void commandBminus1() {
	if ((b - 1) > a) {
		b = b - 1;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres dolny nie mo?e przekraczac %d!\r\n", a + 1);
		UART_SendData(data, size);
	}
}      //zakres gorny -1
void commandBminus10() {
	if ((b - 10) > a) {
		b = b - 10;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres dolny nie mo?e przekraczac %d!\r\n", a + 1);
		UART_SendData(data, size);
	}
}     //zakres gorny -10
void commandBminus100() {
	if ((b - 100) > a) {
		b = b - 100;
		unsigned char data1[256];
		uint16_t size = sprintf(data1, "Zakres dolny ADC = %d (%.3fV)\r\n", a,
				a * 3.3f / 4096.0f);
		UART_SendData(data1, size);
		unsigned char data2[256];
		size = sprintf(data2, "Zakres gorny ADC = %d (%.3fV)\r\n", b,
				b * 3.3f / 4096.0f);
		UART_SendData(data2, size);
	} else {
		unsigned char data[256];
		uint16_t size = sprintf(data,
				"Zakres dolny nie mo?e przekraczac %d!\r\n", a + 1);
		UART_SendData(data, size);
	}
}    //zakres gorny -100
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
