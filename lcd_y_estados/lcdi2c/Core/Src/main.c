




/* IMPORTANTE!!!!!!!!!!!!!!!!!!


NO ES BUENA IDEA USAR LA CARPETA LCDI2C PARA CONSTRUIR EL RESTO DEL CODIGO

se han dado muchos conflictos en esta carpeta, aquí dejo el código listo para juntarlo con la parte de motor,

pero deberá añadirse a un código principal y en ningún momento utilizarse esta carpeta lcd_y_estados corrupta.

DICHO ESTO

este codigo es funcional y constituye la forma si no final, casi final, de la parte correspondiente a gestión de estados y gestión del adc.

*/


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLAVE_ADDRESS_LCD 0x4E
#define TIMESUP_MS 5000 //de momento!!!
#define BRILLO_lim 300 //cuando recibe luz del laser marca unos 150, en condiciones normales puede marcar entre 700 y 1400
#define NIVEL_MAX 3 //de momento
#define PANTALLAS_S4 3


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;


typedef enum {
    S0, //"reposo"
    S1, //"inicio de juego. espera a un START"
	S2, //"3...2...1" y en caso de que antes se haya fallado o acertado manda mensaje
	S3,//"Motor". comienza a girar el motor?? y comienza también la cuenta atrás
	S4 //"Enhorabuena!"
} Estado_t;

typedef enum{
	Ninguno,
	Azul,
	Externo
} Botones_t;

Botones_t boton_pulsado;

volatile Estado_t estado_actual = S0; //volatile pq va variando
volatile int highscore=0; //puntuacion maxima

volatile uint32_t tiempo_inicio = 0;
volatile int paso_cuenta = 0;

volatile int fallo_acierto=0; //podra ser -1 (fallo), 0 (inicio) o 1 (acierto)

volatile uint32_t tiempo_inicio_motor = 0;
volatile uint8_t flag_motor_empieza = 0;

//valores para la parte de fotodiodo
volatile uint32_t valor_adc = 0; //deberia cambiar el nombre
volatile uint8_t detectado = 0;

//variable para los niveles
volatile int nivel=1;


//variables para el highest score
volatile uint32_t tiempo_partida=0; //el tiempo actual de la partida
volatile uint32_t mejor_tiempo=1000000; //uno alto que se supere seguro
volatile int pantalla_final;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Cambio_de_Estados(Botones_t boton_pulsado);
void Reset_S1();
uint32_t Comparar_Tiempos();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void lcd_send_cmd (char cmd)
{
    char data_u, data_l;
    data_u = (cmd & 0xF0);           // extract upper 4 bits
    data_l = ((cmd << 4) & 0xF0);    // extract lower 4 bits

    uint8_t data_t[4];

    // send upper 4 bits with enable pulse
    data_t[0] = data_u | 0x0C;   // EN=1, RS=0  -> bxxxx1100
    data_t[1] = data_u | 0x08;   // EN=0, RS=0  -> bxxxx1000

    // send lower 4 bits with enable pulse
    data_t[2] = data_l | 0x0C;   // EN=1, RS=0  -> bxxxx1100
    data_t[3] = data_l | 0x08;   // EN=0, RS=0  -> bxxxx1000

    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1 -> bxxxx1101
	data_t[1] = data_u|0x09;  //en=0, rs=1 -> bxxxx1001
	data_t[2] = data_l|0x0D;  //en=1, rs=1 -> bxxxx1101
	data_t[3] = data_l|0x09;  //en=0, rs=1 -> bxxxx1001
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_init (void)
{
  // 4 bit initialisation
  HAL_Delay(50);  // wait for >40ms
  lcd_send_cmd (0x30);
  HAL_Delay(5);  // wait for >4.1ms
  lcd_send_cmd (0x30);
  HAL_Delay(1);  // wait for >100us
  lcd_send_cmd (0x30);
  HAL_Delay(10);
  lcd_send_cmd (0x20);  // 4bit mode
  HAL_Delay(10);

  // display initialisation
  lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
  HAL_Delay(1);
  lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
  HAL_Delay(1);
  lcd_send_cmd (0x01);  // clear display
  HAL_Delay(2);
  lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
  HAL_Delay(1);
  lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
  while (*str) lcd_send_data (*str++);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd (col);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x01);
	HAL_Delay(2);
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	static uint32_t last_tick_azul = 0;
	static uint32_t last_tick_externo = 0;

	if (GPIO_Pin == GPIO_PIN_0) {
	        if (HAL_GetTick() - last_tick_azul > 250) { //antirrebote del boton
	        	Cambio_de_Estados(Azul);
	        	last_tick_azul = HAL_GetTick();
	        }
	}

	if (GPIO_Pin == GPIO_PIN_1) { //boton externo
		        if (HAL_GetTick() - last_tick_externo > 250) { //antirrebote del boton
		        	Cambio_de_Estados(Externo);
		        	last_tick_externo = HAL_GetTick();
		        }
		}
}


void Cambio_de_Estados(Botones_t boton_pulsado){

	switch(estado_actual){
	case S0:
		//cualquier boton llama al cambio
		estado_actual=S1;
		break;
	case S1:
		if(boton_pulsado == Externo) estado_actual=S2;
		break;
	case S2:
		if(boton_pulsado == Azul) estado_actual=S1;
		else if (boton_pulsado == Ninguno)estado_actual=S3;
		break;
	case S3:
		if(boton_pulsado == Azul) estado_actual=S1;
		else if(boton_pulsado == Ninguno && nivel == NIVEL_MAX) estado_actual= S4; //igual que o mayor que?
		else if(boton_pulsado == Ninguno) estado_actual =S2;
		break;
	case S4:
		if(boton_pulsado == Azul) estado_actual=S1;
		else if(boton_pulsado == Externo && pantalla_final <PANTALLAS_S4) pantalla_final ++;
		break;
	}
}

void Acciones_Estados(){

	//char buffer[20]; //la voy a borrar seguramente


	switch (estado_actual) {
		          case S0:
		        	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		        	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
		        	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
		        	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
		              lcd_put_cur(0, 0);
		              lcd_send_string("apagado...");
		              lcd_put_cur(1, 0);
		              lcd_send_string("***");

		              break;

		          case S1:
		        	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
		        	  lcd_put_cur(0,0);
		        	  lcd_send_string("*.*.*.Tiro.*.*.*");
		        	  lcd_put_cur(1,0);
		        	  lcd_send_string("*.*a la diana*.*");

		        	  break;
		          case S2:
		        	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
		        	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET); //por si vengo de s3

		        	  if (paso_cuenta == 0) {
		        		  lcd_clear();
		        		  lcd_put_cur(0,0);

		        		  if(fallo_acierto==1) lcd_send_string("Acertaste!");
		        		  else if(fallo_acierto==-1) lcd_send_string("Fallaste!");
		        		  lcd_put_cur(1,0);
		        		  lcd_send_string("3...");
		        		  tiempo_inicio = HAL_GetTick();
		        		  paso_cuenta = 3;
		        	  }
		        	 if(HAL_GetTick() - tiempo_inicio >=1000){
		        		 tiempo_inicio= HAL_GetTick();
		        		 if (paso_cuenta == 3) {
		        			 lcd_send_string("2...");
		        			 tiempo_inicio = HAL_GetTick();
		        			 paso_cuenta = 2;
		        		 }
		        		 else if (paso_cuenta == 2) {
		        			 lcd_send_string("1...");
		        			 tiempo_inicio = HAL_GetTick();
		        			 paso_cuenta = 1;
		        		 }
		        		 else if (paso_cuenta == 1) {
		        			 lcd_clear();
		        			 paso_cuenta = 0;
		        			 //cambiar luego
		        			 Cambio_de_Estados(Ninguno);
		        		 }

		        	 }

		        	 break;

		          case S3:
		        	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);

		        	  if(flag_motor_empieza==0){
		        		  lcd_clear();
		        		  lcd_put_cur(0,0);
		        		  lcd_send_string("tiempo-------->");

		        		  tiempo_inicio_motor=HAL_GetTick();
		        		  flag_motor_empieza=1; // aqui o mejor en el anterior?

		        	  }

		        	  uint32_t tiempo = HAL_GetTick();
		        	  uint32_t transcurrido = tiempo - tiempo_inicio_motor;
		        	  if(transcurrido < TIMESUP_MS){
		        		  char buf[10];
		        		  uint32_t segundos_restantes = (TIMESUP_MS - transcurrido) / 1000;
		        		  lcd_put_cur(1, 0);
		        		  sprintf(buf, "%lu s  ", (segundos_restantes+1));
		        		  lcd_send_string(buf);

		        		  //acierto en la diana
		        		  if(valor_adc < BRILLO_lim){
		        			  detectado =1;
		        			  fallo_acierto = 1;
		        			  flag_motor_empieza=0;
		        			  fallo_acierto=1;
		        			  tiempo_partida += transcurrido;
		        			  nivel++;
		        			  Cambio_de_Estados(Ninguno);
		        		  }

		        	  }
		        	  else {
		        		  flag_motor_empieza = 0;
		        		  fallo_acierto=-1;
		        		  tiempo_partida += transcurrido;
		        		  nivel=0; //que se reinicie cuando falles que si no es muy facil
		        		  Cambio_de_Estados(Ninguno);
		        	  }

		        	  break;
		          case S4:
		        	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
		        	  char puntuacion[16];
		        	  char mejor_puntuacion[16];
		        	  float t_partida = (float)tiempo_partida/1000.0f;
		        	  float mejor_t = (float)mejor_tiempo/1000.0f;
		        	  sprintf(puntuacion, "Tiempo: %.1fs   ", t_partida);
		        	  sprintf(mejor_puntuacion, "Record: %.1fs   ", mejor_t);

		        	  switch(pantalla_final){

		        	  case 0:
		        		  lcd_put_cur(0,0);
		        		  lcd_send_string("*.*.*.*.*.*.*.*");
		        		  lcd_put_cur(1,0);
		        		  lcd_send_string("--ENHORABUENA--");
		        		  break;
		        	  case 1:
		        		  lcd_put_cur(0,0);
		        		  lcd_send_string("--ENHORABUENA--");
		        		  lcd_put_cur(1,0);
		        		  lcd_send_string(puntuacion);
		        		  break;
		        	  case 2:
		        		  lcd_put_cur(0,0);
		        		  lcd_send_string(puntuacion);
		        		  lcd_put_cur(1,0);
		        		  lcd_send_string(mejor_puntuacion);
		        		  break;


		        	  break;

		        	  }

		      }
}

void Reset_S1(){
	//en S1 se hace una limpieza de los valores de las variables
	paso_cuenta=0;
	fallo_acierto=0;
	detectado=0;
	nivel=0;
	flag_motor_empieza=0;
	tiempo_partida=0;
	pantalla_final=0;

	//reset de leds
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);


}


uint32_t Comparar_Tiempos(){

	if(tiempo_partida < mejor_tiempo)
		mejor_tiempo= tiempo_partida;

	return mejor_tiempo;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Stop_DMA(&hadc1);//fuerzo parada por si habia bloqueos
  //arranco el adc en modo dma para que apunte al valor medido
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&valor_adc, 1) != HAL_OK){
	  Error_Handler();
  }

  // Iniciar lcd
  lcd_init ();
  lcd_clear();
  static Estado_t estado_anterior = -1; //es -1 para que no valga ninguno de mis estados

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //anti bloqueo del adc
	  if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR)) {
	            __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
	            HAL_ADC_Stop_DMA(&hadc1);
	            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&valor_adc, 1);
	        }



	  if (estado_actual != estado_anterior) {
	      lcd_clear();//para limpiar la pantalla

	      if(estado_actual == S1) Reset_S1(); //lo llamo desde aqui
	      if (estado_actual == S2) paso_cuenta = 0;

	      if(estado_actual == S4) Comparar_Tiempos();

	      estado_anterior = estado_actual;
	  }

      Acciones_Estados();

      HAL_Delay(20); //maybe lo cambiare


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
